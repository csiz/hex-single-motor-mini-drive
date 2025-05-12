
function mean(values) {
  if (values.length === 0) return 0;
  return values.reduce((a, b) => a + b, 0) / values.length;
}

export function circular_stats_degrees(values){
  const mean_point = [
    mean(values, (d) => Math.cos(d * Math.PI / 180.0)),
    mean(values, (d) => Math.sin(d * Math.PI / 180.0)),
  ]
  const circular_mean = Math.atan2(mean_point[1], mean_point[0]) * 180 / Math.PI;

  const circular_std = Math.sqrt(
    mean(values, (d) => {
      const diff = shortest_distance_degrees(circular_mean, d);
      return diff * diff;
    })
  );

  return {
    circular_mean,
    circular_std,
  };
}

export function radians_to_degrees(radians){
  if (radians === undefined || radians === null) return radians;
  return radians * 180 / Math.PI;
}

export function normalize_degrees(a){
  return (a % 360 + 540) % 360 - 180;
}

export function shortest_distance_degrees(a, b){
  return normalize_degrees(b - a);
}


export function interpolate_degrees(a, b, fraction){
  const diff = b - a;
  return normalize_degrees(a + diff * fraction);
}