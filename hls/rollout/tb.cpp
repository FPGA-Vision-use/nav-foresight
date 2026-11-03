// rollout 的 C 仿真测试台：固定初值跑一次，导出 43×400 点 + 43 个可行标志。
// 与 128 位写版本配套：点按 4 个一组的 128 位字存放，标志在尾部（byte 68800）。
#include <cstdio>
#include <cstdlib>
#include "ap_int.h"

#define K 43
#define N 400

void rollout(float x0, float y0, float ch0, float sh0, float v0, float k0,
             ap_uint<128>* pts);

int main() {
  const float x0 = 1.106652f, y0 = -1.672124f, h0 = 1.602413f, v0 = 0.0f;
  const float ch0 = -0.031611406f, sh0 = 0.999500235f;  // cos/sin(1.602413)

  static ap_uint<128> pts[K * N / 4 + 11];
  rollout(x0, y0, ch0, sh0, v0, 0.0f, pts);   // csim 初值曲率 0

  FILE* f = fopen("rollout_dump.txt", "w");
  if (!f) return 1;
  for (int k = 0; k < K; k++) {
    for (int i = 0; i < N; i++) {
      ap_uint<128> word = pts[(k * N + i) / 4];
      unsigned w = (unsigned)word.range(32 * (i % 4) + 31, 32 * (i % 4));
      short xm = (short)(w & 0xFFFF);
      short ym = (short)(w >> 16);
      fprintf(f, "%d %d %d %d\n", k, i, (int)xm, (int)ym);
    }
  }
  for (int k = 0; k < K; k++) {
    ap_uint<128> vw = pts[K * N / 4 + k / 4];
    unsigned fl = (unsigned)vw.range(32 * (k % 4) + 31, 32 * (k % 4));
    fprintf(f, "V %d %d\n", k, (int)fl);
  }
  fclose(f);
  printf("dump written: %d lines\n", K * N + K);
  return 0;
}
