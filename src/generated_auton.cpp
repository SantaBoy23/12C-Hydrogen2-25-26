// // ===========================================
// // AUTO-GENERATED PURE PURSUIT ROUTINE
// // ===========================================
// void generated_skills_auto() {
//   // 1. Set Starting Position
//   chassis.odom_xyt_set(0.00_in, 0.00_in, 0.00_deg);

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{0.00_in, 0.00_in, 360.00_deg}, fwd, 110},
//     {{0.00_in, -0.00_in, 0.00_deg}, fwd, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-0.58_in, -1.71_in, 10.09_deg}, rev, 110}
//   }, true);
//   chassis.pid_wait();

//   // Mechanism Change
//   intakeTop.move(-110);
//   intakeBottom.move(200);

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-1.52_in, -4.84_in, 13.75_deg}, rev, 110},
//     {{-2.31_in, -7.94_in, 14.65_deg}, rev, 110},
//     {{-3.38_in, -10.95_in, 21.03_deg}, rev, 110},
//     {{-4.87_in, -13.81_in, 27.28_deg}, rev, 110},
//     {{-6.38_in, -16.63_in, 27.61_deg}, rev, 110},
//     {{-7.71_in, -19.40_in, 25.24_deg}, rev, 110},
//     {{-8.94_in, -22.35_in, 22.74_deg}, rev, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-10.07_in, -25.27_in, 21.73_deg}, rev, 110}
//   }, true);
//   chassis.pid_wait();

//   // Mechanism Change
//   intakeTop.move(0);
//   intakeBottom.move(0);

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-11.41_in, -28.15_in, 27.25_deg}, fwd, 110},
//     {{-12.13_in, -28.41_in, 38.56_deg}, fwd, 110},
//     {{-12.61_in, -28.13_in, 48.63_deg}, fwd, 110},
//     {{-12.97_in, -27.65_in, 60.49_deg}, fwd, 110},
//     {{-13.25_in, -26.77_in, 71.30_deg}, fwd, 110},
//     {{-13.29_in, -26.12_in, 82.53_deg}, fwd, 110},
//     {{-13.21_in, -25.33_in, 92.77_deg}, fwd, 110},
//     {{-13.01_in, -24.69_in, 103.03_deg}, fwd, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-13.06_in, -24.16_in, 113.64_deg}, rev, 110},
//     {{-15.89_in, -22.47_in, 119.40_deg}, rev, 110},
//     {{-18.52_in, -20.96_in, 120.07_deg}, rev, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-21.22_in, -19.28_in, 122.66_deg}, rev, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-23.79_in, -17.29_in, 129.50_deg}, fwd, 110},
//     {{-23.06_in, -17.42_in, 139.57_deg}, fwd, 110},
//     {{-21.88_in, -18.26_in, 150.34_deg}, fwd, 110},
//     {{-21.04_in, -19.47_in, 160.94_deg}, fwd, 110},
//     {{-20.49_in, -20.61_in, 171.13_deg}, fwd, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-21.24_in, -17.53_in, 167.13_deg}, rev, 110},
//     {{-22.23_in, -14.48_in, 164.41_deg}, rev, 110},
//     {{-23.42_in, -11.61_in, 161.67_deg}, rev, 110},
//     {{-24.52_in, -8.77_in, 159.87_deg}, rev, 110},
//     {{-25.70_in, -5.76_in, 158.63_deg}, rev, 110},
//     {{-26.84_in, -2.96_in, 157.70_deg}, rev, 110},
//     {{-28.02_in, -0.15_in, 157.12_deg}, rev, 110}
//   }, true);
//   chassis.pid_wait();

//   // Mechanism Change
//   matchLoad.set(true);

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-29.38_in, 2.95_in, 156.42_deg}, rev, 110},
//     {{-30.74_in, 6.03_in, 156.15_deg}, rev, 110},
//     {{-31.95_in, 8.79_in, 156.43_deg}, rev, 110}
//   }, true);
//   chassis.pid_wait();

//   // Mechanism Change
//   intakeTop.move(-110);
//   intakeBottom.move(200);

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-33.04_in, 11.74_in, 160.75_deg}, fwd, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-33.43_in, 14.49_in, 170.99_deg}, fwd, 110}
//   }, true);
//   chassis.pid_wait();

//   // Mechanism Change
//   intakeTop.move(0);
//   intakeBottom.move(0);

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-32.98_in, 11.30_in, 177.81_deg}, fwd, 110},
//     {{-32.92_in, 8.29_in, 179.56_deg}, fwd, 110},
//     {{-33.01_in, 5.21_in, 179.99_deg}, fwd, 110},
//     {{-33.08_in, 2.10_in, 179.77_deg}, fwd, 110},
//     {{-33.09_in, -0.98_in, 179.64_deg}, fwd, 110},
//     {{-33.07_in, -4.03_in, 179.41_deg}, fwd, 110},
//     {{-33.03_in, -7.13_in, 181.01_deg}, fwd, 110}
//   }, true);
//   chassis.pid_wait();

//   // Mechanism Change
//   intakeTop.move(200);
//   intakeBottom.move(200);

//   // Mechanism Change
//   intakeTop.move(0);
//   intakeBottom.move(0);

//   // Mechanism Change
//   matchLoad.set(false);

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-32.63_in, -9.83_in, 191.10_deg}, fwd, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-31.67_in, -7.97_in, 201.17_deg}, rev, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- Batch Move ---
//   chassis.pid_odom_set({
//     {{-30.45_in, -5.20_in, 203.92_deg}, fwd, 110},
//     {{-29.60_in, -2.23_in, 197.62_deg}, fwd, 110},
//     {{-30.31_in, -1.54_in, 187.55_deg}, fwd, 110},
//     {{-30.97_in, -2.11_in, 177.27_deg}, fwd, 110},
//     {{-31.67_in, -3.25_in, 166.81_deg}, fwd, 110},
//     {{-32.22_in, -4.40_in, 156.54_deg}, fwd, 110},
//     {{-32.28_in, -5.60_in, 146.35_deg}, fwd, 110},
//     {{-32.40_in, -7.03_in, 136.01_deg}, fwd, 110},
//     {{-30.02_in, -8.49_in, 146.39_deg}, fwd, 110},
//     {{-29.73_in, -9.48_in, 156.56_deg}, fwd, 110},
//     {{-29.66_in, -9.77_in, 166.91_deg}, fwd, 110},
//     {{-29.61_in, -9.94_in, 177.28_deg}, fwd, 110},
//     {{-29.34_in, -10.10_in, 187.58_deg}, fwd, 110},
//     {{-28.97_in, -10.32_in, 197.89_deg}, fwd, 110},
//     {{-29.41_in, -10.34_in, 187.76_deg}, fwd, 110},
//     {{-29.64_in, -10.71_in, 177.70_deg}, fwd, 110},
//     {{-29.48_in, -13.88_in, 175.79_deg}, fwd, 110}
//   }, true);
//   chassis.pid_wait();

//   // --- End ---
//   chassis.drive_set(0, 0);
//   intakeTop.move(0);
//   intakeBottom.move(0);
// }