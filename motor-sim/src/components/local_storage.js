import _ from 'lodash';

/* Inline copy of https://observablehq.com/@mbostock/safe-local-storage */
class MemoryStorage {
    constructor() {
      Object.defineProperties(this, {_: {value: new Map}});
    }
    get length() {
      return this._.size;
    }
    key(index) {
      return Array.from(this._.keys())[index | 0];
    }
    getItem(key) {
      return this._.has(key += "") ? this._.get(key) : null;
    }
    setItem(key, value) {
      this._.set(key + "", value + "");
    }
    removeItem(key) {
      this._.delete(key + "");
    }
    clear() {
      this._.clear();
    }
  }
  /* Local storage with fallback to in memory storage. */
  export const localStorage = function(){
    try {
      const storage = window.localStorage;
      const key = "__storage_test__";
      storage.setItem(key, key);
      storage.removeItem(key);
      return storage;
    } catch (error) {
      return new MemoryStorage;
    }
  }();
  
  /* Get the stored value or return the default value if not found. */
  export function get_stored_or_default(key, default_value){
    const saved_string = localStorage.getItem(key);
    if (saved_string === null) return default_value;
    const saved_value = JSON.parse(saved_string);
    // Ensure we return all elements of the default value.
    return _.merge({}, default_value, saved_value);
  }