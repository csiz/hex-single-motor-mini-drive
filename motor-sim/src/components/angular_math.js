import * as d3 from 'd3';

export function circular_stats_degrees(values){
  if (!values || values.length === 0) {
    return {
      mean: null,
      stdev: null,
    };
  }

  const mean_point = [
    d3.mean(values, (d) => Math.cos(d * Math.PI / 180.0)),
    d3.mean(values, (d) => Math.sin(d * Math.PI / 180.0)),
  ];
  
  const mean = radians_to_degrees(Math.atan2(mean_point[1], mean_point[0]));

  const stdev = Math.sqrt(
    d3.mean(values, (d) => {
      const diff = signed_distance_degrees(mean, d);
      return diff * diff;
    })
  );

  return {
    mean,
    stdev,
  };
}

export function radians_to_degrees(radians){
  if (radians === undefined || radians === null) return radians;
  return radians * 180 / Math.PI;
}

export function normalize_degrees(a){
  return (a % 360 + 540) % 360 - 180;
}

export function positive_degrees(d){
  return (d % 360 + 360) % 360;
}

export function signed_distance_degrees(a, b){
  return normalize_degrees(b - a);
}

export function positive_distance_degrees(a, b){
  return positive_degrees(b - a);
} 

export function interpolate_degrees(a, b, fraction){
  // We always want to go counter-clockwise, so the difference must be positive.
  const diff = positive_distance_degrees(a, b);
  return normalize_degrees(a + diff * fraction);
}