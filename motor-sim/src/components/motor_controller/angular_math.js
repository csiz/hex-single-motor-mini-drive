
function mean(data, accessor){
  if (!data || data.length === 0) return 0;
  if (!accessor) accessor = (d) => d;
  const sum = data.reduce((acc, d) => acc + accessor(d), 0);
  return sum / data.length;
}

export function circular_stats(values){
  if (!values || values.length === 0) {
    return {
      mean: null,
      stdev: null,
    };
  }

  const mean_point = [
    mean(values, (d) => Math.cos(d)),
    mean(values, (d) => Math.sin(d)),
  ];
  
  const mean = Math.atan2(mean_point[1], mean_point[0]);

  const stdev = Math.sqrt(
    mean(values, (d) => {
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