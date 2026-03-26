// #include "main.h"
// #include <cmath>
// #include <numbers>

// struct XorShift32 {
//   uint32_t state;

//   inline XorShift32(uint32_t seed = pros::micros())
//     : state(seed == 0 ? 0x12345678 : seed) {}

//   inline uint32_t next_u32() {
//     uint32_t x = state;
//     x ^= x << 13;
//     x ^= x >> 17;
//     x ^= x << 5;
//     state = x;
//     return x;
//   }

//   inline float next_f32() {
//     return (next_u32() >> 8) * (1.0f / (1u << 24));
//   }

//   inline float range_f32(float min, float max) {
//     return min + (max - min) * next_f32();
//   }

//   // box-muller transform, use ziggurat for optimization if you want
//   inline float gaussian(float std_dev) {
//     float u1 = std::max(next_f32(), 1e-12f);
//     float u2 = next_f32();
//     float r = std::sqrt(-2.0f * std::log(u1));
//     float theta = 2.0f * M_PI * u2;
//     return r * std::cos(theta) * std_dev;
//   }
// };

// // Official VEX Competition Field size in inches
// const float FIELD_SIZE = 140.42f;
// const float HALF_SIZE = 0.5f * FIELD_SIZE;
// const float FIELD_MIN = -HALF_SIZE;
// const float FIELD_MAX = HALF_SIZE;

// template <size_t N>
// struct MCL {
//   float particle_x[N];
//   float particle_y[N];
//   float particle_weights[N];

//   float temp_x[N];
//   float temp_y[N];
//   float temp_weights[N];

//   float presample_x[N];
//   float presample_y[N];
//   float presample_weights[N];

//   XorShift32 rng;

//   MCL() : rng(pros::micros()) {
//     std::fill(particle_x, particle_x + N, 0.0f);
//     std::fill(particle_y, particle_y + N, 0.0f);
//     std::fill(particle_weights, particle_weights + N, 1.0f / N);
//     std::fill(temp_x, temp_x + N, 0.0f);
//     std::fill(temp_y, temp_y + N, 0.0f);
//     std::fill(temp_weights, temp_weights + N, 0.0f);
//     std::fill(presample_x, presample_x + N, 0.0f);
//     std::fill(presample_y, presample_y + N, 0.0f);
//     std::fill(presample_weights, presample_weights + N, 0.0f);
//   }
// };

// struct Reading {
//   float recorded;
//   float inv_var;

//   Point relative_pos;
//   Point proj_relative;

//   Reading(float recorded, float std_dev, Point relative_pos, Point proj_relative)
//     : recorded(recorded),
//       inv_var(-0.5f / (std_dev * std_dev)),
//       relative_pos(relative_pos),
//       proj_relative(proj_relative) {}
// };

// template <size_t N>
// void MCL<N>::init(float x, float y, float spread) {
//   rng = XorShift32(pros::micros());

//   for (size_t i = 0; i < N; i++) {
//     particle_x[i] = std::clamp(x + rng.range_f32(-spread, spread), FIELD_MIN, FIELD_MAX);
//     particle_y[i] = std::clamp(y + rng.range_f32(-spread, spread), FIELD_MIN, FIELD_MAX);
//     particle_weights[i] = 1.0f / N;
//   }
// }

