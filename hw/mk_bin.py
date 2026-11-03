"""把 pts.txt 转成板子直吃的二进制。

pts.txt 格式（tb_clear.cpp 的输入）：
    第 1 行：20 个整数参数（顺序见 INTERFACE.md 第 5 节，最后一个是 0 占位）
    之后 2150 行：每行 8 个 hex，是一个 256 位字里的 8 个 32 位小端字

输出：
    pts_u32.bin   17200 个 int32 小端 = 68800 B，PL 直接按 256 位读
    params.json   19 个参数名 → 值
    ref.txt       csim 参考 margin（若存在 margins.txt）
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
NAMES = ["xmin", "ymin", "xmax", "ymax", "pcx", "pcy", "psx", "psy",
         "acx", "acy", "avx", "avy", "abase", "ccx", "ccy", "cvx", "cvy",
         "crad", "margin_mm", "sw_guard"]


def main(src=None, outdir=None):
    src = src or os.path.join(ROOT, "pts.txt")
    outdir = outdir or HERE
    lines = [l.split() for l in open(src) if l.strip()]
    p = [int(x) for x in lines[0]]
    if len(p) < len(NAMES):
        raise SystemExit("参数不足: %d" % len(p))
    params = {n: p[i] for i, n in enumerate(NAMES)
              if i < len(p) and n != "sw_guard"}
    params["sw_guard"] = 350      # v1.3 新增：sweep 半径常量，必须 350
    body = lines[1:]
    if len(body) != K_WORDS:
        raise SystemExit("字数不对: %d != %d" % (len(body), K_WORDS))
    raw = bytearray()
    for row in body:
        if len(row) != 8:
            raise SystemExit("行宽不对: %r" % (row,))
        for h in row:
            raw += int(h, 16).to_bytes(4, "little")
    open(os.path.join(outdir, "pts_u32.bin"), "wb").write(raw)
    json.dump(params, open(os.path.join(outdir, "params.json"), "w"), indent=1)
    ref = os.path.join(ROOT, "margins.txt")
    if os.path.exists(ref):
        open(os.path.join(outdir, "ref.txt"), "w").write(
            open(ref).read().strip() + "\n")
    print("pts_u32.bin %d B, params %s" % (len(raw), params))


K_WORDS = 43 * 400 // 8

if __name__ == "__main__":
    main(*(sys.argv[1:3]))
