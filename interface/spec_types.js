
function assemble_literal(strings, ...values) {
  // Reassemble or process the template; or just use the string if
  // strings is not an array (normal function call).
  return Array.isArray(strings)
    ? strings.reduce((acc, str, i) => acc + str + (values[i] ?? ""), "")
    : strings;
}

export class CodeWriter {
  constructor() {
    this.code = '';
    this.indent_level = 0;
  }
  write(strings, ...values) {
    const string = assemble_literal(strings, ...values);

    this.code += '  '.repeat(this.indent_level) + string + '\n';
  }
  comment(strings, ...values) {
    const string = assemble_literal(strings, ...values);
    const comment_lines = string.trim().split('\n');
    for (const line of comment_lines) {
      this.code += '  '.repeat(this.indent_level) + `// ${line}` + '\n';
    }
  }
  indent() {
    this.indent_level += 1;
  }
  dedent() {
    this.indent_level = Math.max(0, this.indent_level - 1);
  }
  get() {
    return this.code;
  }
}

export class ConstantSpec {
  doc;
  name;
  type;
  const;

  constructor(options){Object.assign(this, options)};
}

export class TypeSpec {
  doc;
  name;
  size;

  constructor(options){Object.assign(this, options)};
}

export class BasicType {
  name;
  size;
  cpp_type;
  js_type;
  cpp_serialization;
  cpp_deserialization;
  js_serialization;
  js_deserialization;

  constructor(options){Object.assign(this, options)};
}


export class ArraySpec extends TypeSpec {
  type;
  array_len;

  constructor(options){super(options);Object.assign(this, options)};
}

export class StructSpec extends TypeSpec {
  struct;
  struct_base;

  constructor(options){super(options);Object.assign(this, options)};
}

export class FieldSpec {
  doc;
  name;
  type;

  constructor(options){Object.assign(this, options)};
}

export class MessageSpec {
  doc;
  name;
  message_code;
  type;
  size;

  constructor(options){Object.assign(this, options)};
};

export class Spec {

  doc;
  definitions;
  message_codes;

  constructor(options){Object.assign(this, options)};
};