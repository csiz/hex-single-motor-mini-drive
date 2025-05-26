import normal_dist from "@stdlib/stats-base-dists-normal";

export const pdf_normal = normal_dist.pdf;
export const cdf_normal = normal_dist.cdf;

export function product_of_normals({mean_a, stdev_a, mean_b, stdev_b}){
  const mean = (mean_a * stdev_b * stdev_b + mean_b * stdev_a * stdev_a) / (stdev_a * stdev_a + stdev_b * stdev_b);
  const stdev = Math.sqrt((stdev_a * stdev_a * stdev_b * stdev_b) / (stdev_a * stdev_a + stdev_b * stdev_b));
  return {mean, stdev};
}

export function product_of_normals_by_variance({mean_a, variance_a, mean_b, variance_b}){
  const mean = (mean_a * variance_b + mean_b * variance_a) / (variance_a + variance_b);
  const variance = (variance_a * variance_b) / (variance_a + variance_b);
  return {mean, variance};
}

export function add_stdev(...std_values){
  return Math.sqrt(std_values.reduce((sum, stdev) => sum + stdev * stdev, 0));
}