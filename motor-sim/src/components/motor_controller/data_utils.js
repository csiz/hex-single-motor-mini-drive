import * as _ from 'lodash';


export function online_map(array, online_fn){
  if (array.length == 0) return [];

  const result = new Array(array.length);
  let previous = online_fn(array[0], undefined);
  result[0] = previous;
  for (let i = 1; i < array.length; i++){
    previous = online_fn(array[i], previous);
    result[i] = previous;
  }
  return result;
}

export function online_function_chain(...online_functions){
  return function(value, previous){
    for (const fn of online_functions){
      value = fn.call(this, value, previous);
    }
    return value;
  };
}

// Merge records multiple arrays into a single array of records; with 
// properties prefixed by the name of the array they came from.
export function zip_records(prefix_to_array_of_records){
  prefix_to_array_of_records = Object.entries(prefix_to_array_of_records);
  
  if (prefix_to_array_of_records.length == 0) return [];

  const longest_length = Math.max(...prefix_to_array_of_records.map(([prefix, array]) => array.length));

  return Array.from({length: longest_length}, (_unused, i) => {
    let result = {};
    for (const [prefix, array] of prefix_to_array_of_records) {
      const record = array[i];
      if (record === undefined) continue; // Skip if the record is undefined.
      if (typeof record === "object" && record !== null) {
        // If the record is an object, prefix its keys.
        Object.entries(record).forEach(([key, value]) => {
          result[`${prefix}_${key}`] = value;
        });
      } else {
        // If the record is a primitive value, just use the prefix.
        result[prefix] = record;
      }
    }
    return result;
  });
}
