import normal_dist from "@stdlib/stats-base-dists-normal";

export const pdf_normal = normal_dist.pdf;
export const cdf_normal = normal_dist.cdf;
export const quantile_normal = normal_dist.quantile;

export function product_of_normals({mean_a, stdev_a, mean_b, stdev_b}){
  if (stdev_a <= 0 || stdev_b <= 0) {
    throw new Error("Standard deviations must be strictly positive");
  }

  const mean = (mean_a * stdev_b * stdev_b + mean_b * stdev_a * stdev_a) / (stdev_a * stdev_a + stdev_b * stdev_b);
  const stdev = Math.sqrt((stdev_a * stdev_a * stdev_b * stdev_b) / (stdev_a * stdev_a + stdev_b * stdev_b));
  return {mean, stdev};
}

export function product_of_normals_by_variance({mean_a, variance_a, mean_b, variance_b}){
  if (variance_a <= 0 || variance_b <= 0) {
    throw new Error("Variances must be strictly positive");
  }
  const mean = (mean_a * variance_b + mean_b * variance_a) / (variance_a + variance_b);
  const variance = (variance_a * variance_b) / (variance_a + variance_b);
  return {mean, variance};
}

export function add_stdev(...std_values){
  return Math.sqrt(std_values.reduce((sum, stdev) => sum + stdev * stdev, 0));
}

export function weighted_product_of_normals({mean_a, stdev_a, weight_a, mean_b, stdev_b, weight_b}) {

  if (stdev_a <= 0 || stdev_b <= 0) {
    throw new Error("Standard deviations must be strictly positive");
  }

  const {mean, variance} = weighted_product_of_normals_by_variance({
    mean_a,
    variance_a: stdev_a * stdev_a,
    weight_a,
    mean_b,
    variance_b: stdev_b * stdev_b,
    weight_b,
  });

  return {
    mean,
    stdev: Math.sqrt(variance),
  };
}

export function weighted_product_of_normals_by_variance({mean_a, variance_a, weight_a, mean_b, variance_b, weight_b}) {
  if (weight_a < 0 || weight_a > 1 || weight_b < 0 || weight_b > 1) {
    throw new Error("Weights must be in the range [0, 1]");
  }
  if (variance_a <= 0 || variance_b <= 0) {
    throw new Error("Variances must be strictly positive");
  }

  const adjusted_variance_a = variance_a * weight_b;
  const adjusted_variance_b = variance_b * weight_a;
  const adjusted_variance_sum = adjusted_variance_a + adjusted_variance_b;

  const variance = variance_a * variance_b / adjusted_variance_sum;
  const mean = (mean_a * adjusted_variance_b + mean_b * adjusted_variance_a) / adjusted_variance_sum;

  return {mean, variance};
}
