import * as d3 from 'd3';

export function circular_stats(values){
  if (!values || values.length === 0) {
    return {
      mean: null,
      stdev: null,
    };
  }

  const mean_point = [
    d3.mean(values, (d) => Math.cos(d)),
    d3.mean(values, (d) => Math.sin(d)),
  ];
  
  const mean = Math.atan2(mean_point[1], mean_point[0]);

  const stdev = Math.sqrt(
    d3.mean(values, (d) => {
      const diff = normalize_radians(d - mean);
      return diff * diff;
    })
  );

  return {
    mean,
    stdev,
  };
}

export function normalize_radians(a){
  return (a % (2 * Math.PI) + 3 * Math.PI) % (2 * Math.PI) - Math.PI;
}

export function positive_radians(d){
  return (d % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
}


export function interpolate_radians(a, b, fraction){
  // We always want to go counter-clockwise, so the difference must be positive.
  const diff = positive_radians(b - a);
  return normalize_radians(a + diff * fraction);
}