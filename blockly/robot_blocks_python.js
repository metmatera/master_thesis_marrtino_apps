Blockly.Python['begin'] = function(block) {
  var code = 'begin()\n';
  return code;
};

Blockly.Python['end'] = function(block) {
  var code = 'end()\n';
  return code;
};

Blockly.Python['forward'] = function(block) {
  var value_steps = Blockly.Python.valueToCode(block, 'steps', Blockly.Python.ORDER_ATOMIC);
  var code = 'forward('+value_steps+')\n';
  return code;
};

Blockly.Python['backward'] = function(block) {
  var value_steps = Blockly.Python.valueToCode(block, 'steps', Blockly.Python.ORDER_ATOMIC);
  var code = 'backward('+value_steps+')\n';
  return code;
};

Blockly.Python['left'] = function(block) {
  var value_steps = Blockly.Python.valueToCode(block, 'steps', Blockly.Python.ORDER_ATOMIC);
  var code = 'left('+value_steps+')\n';
  return code;
};

Blockly.Python['right'] = function(block) {
  var value_steps = Blockly.Python.valueToCode(block, 'steps', Blockly.Python.ORDER_ATOMIC);
  var code = 'right('+value_steps+')\n';
  return code;
};

