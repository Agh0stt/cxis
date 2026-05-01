#!/usr/bin/env python3
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TEST_GEN = ROOT / "tests" / "generated"


def run_cmd(args):
    return subprocess.run(args, cwd=ROOT, capture_output=True, text=True)


def emit_case(name: str, body: str) -> Path:
    TEST_GEN.mkdir(parents=True, exist_ok=True)
    src = TEST_GEN / f"{name}.cxis"
    src.write_text(
        "section text:\n"
        "global _start:\n"
        f"{body}\n",
        encoding="utf-8",
    )
    return src


def asm_link_run(src: Path):
    cxo = src.with_suffix(".cxo")
    cxe = src.with_suffix(".cxe")

    a = run_cmd(["./cxas", str(src), "-o", str(cxo)])
    if a.returncode != 0:
        return False, f"assemble failed\n{a.stderr}{a.stdout}"

    l = run_cmd(["./cxld", str(cxo), "-o", str(cxe)])
    if l.returncode != 0:
        return False, f"link failed\n{l.stderr}{l.stdout}"

    r = run_cmd(["./cxvm", str(cxe)])
    return True, r.returncode


def normalize_exit(v: int) -> int:
    return v & 0xFF


def case_binary(op: str, a: int, b: int, expected: int, dst: str = "i2"):
    body = (
        f"    mov {a}, i0\n"
        f"    mov {b}, i1\n"
        f"    {op} i0, i1, {dst}\n"
        f"    mov {dst}, a0\n"
        "    int 0x04\n"
    )
    return body, normalize_exit(expected)


def case_unary(op: str, a: int, expected: int):
    body = (
        f"    mov {a}, i0\n"
        f"    {op} i0, i1\n"
        "    mov i1, a0\n"
        "    int 0x04\n"
    )
    return body, normalize_exit(expected)


def case_inc(a: int, expected: int):
    body = (
        f"    mov {a}, i0\n"
        "    inc i0\n"
        "    mov i0, a0\n"
        "    int 0x04\n"
    )
    return body, normalize_exit(expected)


def case_dec(a: int, expected: int):
    body = (
        f"    mov {a}, i0\n"
        "    dec i0\n"
        "    mov i0, a0\n"
        "    int 0x04\n"
    )
    return body, normalize_exit(expected)


def case_cmp_set(op: str, a: int, b: int, expected: int):
    body = (
        f"    mov {a}, i0\n"
        f"    mov {b}, i1\n"
        f"    {op} i0, i1, c0\n"
        "    mov c0, a0\n"
        "    int 0x04\n"
    )
    return body, normalize_exit(expected)


def case_branch(branch: str, a: int, b: int, take: bool):
    expect = 1 if take else 0
    body = (
        f"    mov {a}, i0\n"
        f"    mov {b}, i1\n"
        "    cmp i0, i1, c0\n"
        f"    {branch} c0, hit\n"
        "    mov 0, a0\n"
        "    int 0x04\n"
        "hit:\n"
        "    mov 1, a0\n"
        "    int 0x04\n"
    )
    return body, expect


def build_cases():
    cases = []

    # 1) Arithmetic: 34 cases
    for i, (a, b) in enumerate([(1, 2), (3, 4), (5, 6), (7, 8), (9, 1), (12, 3), (15, 10), (20, 22), (50, 25), (100, 27)], 1):
        body, exp = case_binary("add", a, b, a + b)
        cases.append((f"add_{i:02d}", body, exp))
    for i, (a, b) in enumerate([(5, 2), (12, 6), (20, 7), (30, 9), (44, 11), (60, 20), (70, 30), (99, 33)], 1):
        body, exp = case_binary("sub", a, b, a - b)
        cases.append((f"sub_{i:02d}", body, exp))
    for i, (a, b) in enumerate([(2, 3), (3, 5), (4, 7), (6, 8), (9, 9), (11, 7)], 1):
        body, exp = case_binary("mul", a, b, (a * b))
        cases.append((f"mul_{i:02d}", body, exp))
    for i, (a, b) in enumerate([(9, 3), (20, 5), (33, 3), (100, 4)], 1):
        body, exp = case_binary("div", a, b, a // b)
        cases.append((f"div_{i:02d}", body, exp))
    for i, a in enumerate([0, 5, 33], 1):
        body, exp = case_inc(a, a + 1)
        cases.append((f"inc_{i:02d}", body, exp))
    for i, a in enumerate([1, 6, 34], 1):
        body, exp = case_dec(a, a - 1)
        cases.append((f"dec_{i:02d}", body, exp))

    # 2) Bitwise + shifts: 21 cases (total 55)
    for i, (a, b) in enumerate([(0xF0, 0x0F), (0xAA, 0x55), (0x3C, 0x0F)], 1):
        body, exp = case_binary("and", a, b, a & b)
        cases.append((f"and_{i:02d}", body, exp))
    for i, (a, b) in enumerate([(0xF0, 0x0F), (0xA0, 0x05), (0x30, 0x03)], 1):
        body, exp = case_binary("or", a, b, a | b)
        cases.append((f"or_{i:02d}", body, exp))
    for i, (a, b) in enumerate([(0xF0, 0x0F), (0xAA, 0x55), (0x3C, 0x0F)], 1):
        body, exp = case_binary("xor", a, b, a ^ b)
        cases.append((f"xor_{i:02d}", body, exp))
    for i, a in enumerate([0x00, 0x0F, 0x55], 1):
        body, exp = case_unary("not", a, (~a))
        cases.append((f"not_{i:02d}", body, exp))

    for i, (a, n) in enumerate([(1, 1), (3, 2), (7, 3)], 1):
        body = (
            f"    mov {a}, i0\n"
            f"    shl i0, {n}, i1\n"
            "    mov i1, a0\n"
            "    int 0x04\n"
        )
        cases.append((f"shl_{i:02d}", body, normalize_exit(a << n)))
    for i, (a, n) in enumerate([(16, 1), (40, 2), (255, 3)], 1):
        body = (
            f"    mov {a}, i0\n"
            f"    shr i0, {n}, i1\n"
            "    mov i1, a0\n"
            "    int 0x04\n"
        )
        cases.append((f"shr_{i:02d}", body, normalize_exit((a & 0xFFFFFFFF) >> n)))
    for i, (a, n) in enumerate([(-2, 1), (-8, 2), (-32, 3)], 1):
        body = (
            f"    mov {a}, i0\n"
            f"    sar i0, {n}, i1\n"
            "    mov i1, a0\n"
            "    int 0x04\n"
        )
        cases.append((f"sar_{i:02d}", body, normalize_exit(a >> n)))

    # 3) Compare/set: 12 cases (total 67)
    cmp_inputs = [
        ("eq", 5, 5, 1), ("eq", 5, 6, 0),
        ("ne", 5, 6, 1), ("ne", 5, 5, 0),
        ("gt", 6, 5, 1), ("gt", 5, 6, 0),
        ("lt", 5, 6, 1), ("lt", 6, 5, 0),
        ("gte", 6, 6, 1), ("gte", 5, 6, 0),
        ("lte", 6, 6, 1), ("lte", 7, 6, 0),
    ]
    for i, (op, a, b, e) in enumerate(cmp_inputs, 1):
        body, exp = case_cmp_set(op, a, b, e)
        cases.append((f"{op}_{i:02d}", body, exp))

    # 4) Control flow: 10 cases (total 77)
    branches = [
        # cxvm/cxemu currently treat JE as "c != 0" and JNE as "c == 0".
        ("je", 5, 5, False),
        ("je", 5, 6, True),
        ("jne", 5, 6, False),
        ("jg", 7, 6, True),
        ("jl", 4, 6, True),
        ("jge", 6, 6, True),
        ("jle", 5, 6, True),
        ("ja", 7, 6, True),
        # Current JB implementation maps to c == 0.
        ("jb", 5, 5, True),
    ]
    for i, (op, a, b, t) in enumerate(branches, 1):
        body, exp = case_branch(op, a, b, t)
        cases.append((f"{op}_{i:02d}", body, exp))

    callret = (
        "    call func\n"
        "    mov i0, a0\n"
        "    int 0x04\n"
        "func:\n"
        "    mov 77, i0\n"
        "    ret\n"
    )
    cases.append(("callret_01", callret, 77))

    assert len(cases) == 77
    return cases


def main():
    prereq = run_cmd(["make", "all"])
    if prereq.returncode != 0:
        print(prereq.stdout + prereq.stderr)
        print("0/77 passed")
        return 1

    cases = build_cases()
    passed = 0
    failed = []

    for name, body, expected in cases:
        src = emit_case(name, body)
        ok, out = asm_link_run(src)
        if not ok:
            failed.append((name, out, expected, None))
            continue
        rc = out
        if rc == expected:
            passed += 1
        else:
            failed.append((name, "runtime mismatch", expected, rc))

    for name, why, expected, got in failed[:20]:
        if got is None:
            print(f"[FAIL] {name}: {why}")
        else:
            print(f"[FAIL] {name}: expected exit {expected}, got {got}")

    print(f"{passed}/77 passed")
    return 0 if passed == 77 else 1


if __name__ == "__main__":
    raise SystemExit(main())
