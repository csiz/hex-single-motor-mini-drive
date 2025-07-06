import { command_codes } from "./motor_interface.js";

export const unit_test_expected = {
  [command_codes.RUN_UNIT_TEST_ATAN]: `\
atan2(100, 0) = (256, 100)
atan2(0, 100) = (0, 100)
atan2(-100, 0) = (768, 100)
atan2(0, -100) = (512, 100)
atan2(100, 100) = (128, 121)
atan2(-100, -100) = (640, 121)
atan2(100, -100) = (384, 121)
atan2(-100, 100) = (896, 121)
`,
  [command_codes.RUN_UNIT_TEST_FUNKY_ATAN]: `\
funky_atan2(100, 0) = 256
funky_atan2(0, 100) = 0
funky_atan2(-100, 0) = -256
funky_atan2(0, -100) = -512
funky_atan2(100, 100) = 102
funky_atan2(-100, -100) = -410
funky_atan2(100, -100) = 358
funky_atan2(-100, 100) = -154
`,
  [command_codes.RUN_UNIT_TEST_FUNKY_ATAN_PART_2]: `\
funky_atan2(10, 100) = 12
funky_atan2(20, 100) = 24
funky_atan2(30, 100) = 35
funky_atan2(40, 100) = 46
funky_atan2(50, 100) = 56
funky_atan2(60, 100) = 66
funky_atan2(70, 100) = 76
funky_atan2(80, 100) = 85
funky_atan2(90, 100) = 94
`,
  [command_codes.RUN_UNIT_TEST_FUNKY_ATAN_PART_3]: `\
f_atan2(-100, 1000) = -12
f_atan2(-200, 1000) = -24
f_atan2(-300, 1000) = -35
f_atan2(-400, 1000) = -46
f_atan2(-500, 1000) = -56
f_atan2(-600, 1000) = -66
f_atan2(-700, 1000) = -76
f_atan2(-800, 1000) = -85
f_atan2(-900, 1000) = -94
`,
};