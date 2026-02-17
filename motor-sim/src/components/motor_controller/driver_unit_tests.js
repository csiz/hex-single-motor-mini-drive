import { MessageCode } from "./motor_controller.js";

export const unit_test_expected = {
  [MessageCode.RUN_UNIT_TEST_FUNKY_ATAN]: `\
funky_atan2(100, 0) = 256
funky_atan2(0, 100) = 0
funky_atan2(-100, 0) = -256
funky_atan2(0, -100) = -512
funky_atan2(100, 100) = 129
funky_atan2(-100, -100) = -383
funky_atan2(100, -100) = 385
funky_atan2(-100, 100) = -127
`,
  [MessageCode.RUN_UNIT_TEST_FUNKY_ATAN_PART2]: `\
funky_atan2(10, 100) = 15
funky_atan2(20, 100) = 30
funky_atan2(30, 100) = 45
funky_atan2(40, 100) = 58
funky_atan2(50, 100) = 72
funky_atan2(60, 100) = 84
funky_atan2(70, 100) = 96
funky_atan2(80, 100) = 108
funky_atan2(90, 100) = 119
`,
  [MessageCode.RUN_UNIT_TEST_FUNKY_ATAN_PART3]: `\
f_atan2(-100, 1000) = -15
f_atan2(-200, 1000) = -30
f_atan2(-300, 1000) = -45
f_atan2(-400, 1000) = -58
f_atan2(-500, 1000) = -72
f_atan2(-600, 1000) = -84
f_atan2(-700, 1000) = -96
f_atan2(-800, 1000) = -108
f_atan2(-900, 1000) = -119
`,
};