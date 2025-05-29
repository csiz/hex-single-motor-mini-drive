import {command_codes} from "./motor_interface.js";  

export const unit_test_atan_expected = `\
atan2(100, 0) = (256, 100)
atan2(0, 100) = (0, 100)
atan2(-100, 0) = (768, 100)
atan2(0, -100) = (512, 100)
atan2(100, 100) = (128, 121)
atan2(-100, -100) = (640, 121)
atan2(100, -100) = (384, 121)
atan2(-100, 100) = (896, 121)
`;

export const unit_test_integer_arithmetic_expected = `\
a = (28098, 0, -12) ~ (686)
b = (25722, 1, -13) ~ (-313)
a + b = (30474, 0, -13) ~ (372)
a - b = (20479, 0, -11) ~ (999)
a * b = (22056, 1, -10) ~ (-2153)
a / b = (17897, 1, -13) ~ (-218)
`;

export const unit_test_expected = {
  [command_codes.RUN_UNIT_TEST_ATAN]: unit_test_atan_expected,
  [command_codes.RUN_UNIT_TEST_INTEGER_ARITHMETIC]: unit_test_integer_arithmetic_expected,
};