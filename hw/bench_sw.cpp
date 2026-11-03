// 纯软件基线：同一套 HLS C 代码在 PC 单线程上跑，测单帧耗时。
// 编译：g++ -O2 -std=c++14 -I <vitis_include> bench_sw.cpp clearance.cpp rollout/rollout.cpp
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include "ap_int.h"

#define K 43
#define N 400

void rollout(float x0, float y0, float ch0, float sh0, float v0, float k0,
             ap_uint<128>* pts);

void clearance(const ap_uint<256>* pts,
               int xmin, int ymin, int xmax, int ymax,
               int pcx, int pcy, int psx, int psy,
               int acx, int acy, int avx, int avy, int abase,
               int ccx, int ccy, int cvx, int cvy, int crad,
               int margin_mm, int sw_guard, int* out);

int main(int argc, char** argv) {
  int iters = (argc > 1) ? atoi(argv[1]) : 100;

  static ap_uint<128> rpts[K * N / 4 + 11];
  static ap_uint<256> cpts[2150];
  static int out[K * 5];

  // 第 0 帧的真实初值与参数（ref_frames.jsonl）
  const float x0 = 1.0012413f, y0 = -1.9616917f;
  const float h0 = 1.5711353f, v0 = 0.0f;
  const float ch0 = 0.0f, sh0 = 1.0f;  // cos/sin(h0) ≈ 0 / 1

  // 预热
  rollout(x0, y0, ch0, sh0, v0, 0.0f, rpts);
  memset(cpts, 0, sizeof(cpts));
  clearance(cpts, 0, -2600, 2600, 2100, -620, 800, 1000, 1000,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 230, 350, out);

  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iters; i++) {
    rollout(x0, y0, ch0, sh0, v0, 0.0f, rpts);
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iters; i++) {
    clearance(cpts, 0, -2600, 2600, 2100, -620, 800, 1000, 1000,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 230, 350, out);
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  double r_ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / iters;
  double c_ms = std::chrono::duration<double, std::milli>(t2 - t1).count() / iters;

  printf("iters = %d\n", iters);
  printf("rollout   : %8.3f ms/frame\n", r_ms);
  printf("clearance : %8.3f ms/frame\n", c_ms);
  printf("total     : %8.3f ms/frame\n", r_ms + c_ms);
  printf("(checksum rollout=%u clearance=%d)\n",
         (unsigned)rpts[0].range(31, 0), out[0]);
  return 0;
}
