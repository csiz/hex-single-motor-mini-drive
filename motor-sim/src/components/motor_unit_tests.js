import { command_codes } from "./motor_interface.js";

export const unit_test_expected = {
  [command_codes.RUN_UNIT_TEST_FUNKY_ATAN]: `\
funky_atan2(100, 0) = 1024
funky_atan2(0, 100) = 0
funky_atan2(-100, 0) = -1024
funky_atan2(0, -100) = -2048
funky_atan2(100, 100) = 520
funky_atan2(-100, -100) = -1528
funky_atan2(100, -100) = 1544
funky_atan2(-100, 100) = -504
`,
  [command_codes.RUN_UNIT_TEST_FUNKY_ATAN_PART_2]: `\
funky_atan2(10, 100) = 63
funky_atan2(20, 100) = 124
funky_atan2(30, 100) = 181
funky_atan2(40, 100) = 236
funky_atan2(50, 100) = 289
funky_atan2(60, 100) = 339
funky_atan2(70, 100) = 387
funky_atan2(80, 100) = 434
funky_atan2(90, 100) = 478
`,
  [command_codes.RUN_UNIT_TEST_FUNKY_ATAN_PART_3]: `\
f_atan2(-100, 1000) = -63
f_atan2(-200, 1000) = -124
f_atan2(-300, 1000) = -181
f_atan2(-400, 1000) = -236
f_atan2(-500, 1000) = -289
f_atan2(-600, 1000) = -339
f_atan2(-700, 1000) = -387
f_atan2(-800, 1000) = -434
f_atan2(-900, 1000) = -478
`,
};