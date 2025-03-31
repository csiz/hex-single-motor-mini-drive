import {normalized_angle, π, phase_switches, phase_states} from "./simulation.js";

function get_sixstep_hall_code(hall_u, hall_v, hall_w){
  return hall_w << 2 | hall_v << 1 | hall_u;
}

const sixstep_phase_map = {
  0b000: "no_magnet",
  0b100: 0,
  0b110: 1,
  0b010: 2,
  0b011: 3,
  0b001: 4,
  0b101: 5,
  0b111: "too_much_magnet",
};

const sixstep_switching_neg_map = {
  no_magnet: [0, 0, 0],
  too_much_magnet: [0, 0, 0],
  0: [0, 1, 2],
  1: [1, 0, 2],
  2: [1, 2, 0],
  3: [0, 2, 1],
  4: [2, 0, 1],
  5: [2, 1, 0],
};

const sixstep_switching_pos_map = {
  no_magnet: [0, 0, 0],
  too_much_magnet: [0, 0, 0],
  0: [0, 2, 1],
  1: [2, 0, 1],
  2: [2, 1, 0],
  3: [0, 1, 2],
  4: [1, 0, 2],
  5: [1, 2, 0],
};

export function sixstep_commutation(outputs, positive=true){
  const {hall_1, hall_2, hall_3} = outputs;
  const sixstep_phase = sixstep_phase_map[get_sixstep_hall_code(hall_1, hall_2, hall_3)];
  return (positive ? sixstep_switching_pos_map : sixstep_switching_neg_map)[sixstep_phase];
}
