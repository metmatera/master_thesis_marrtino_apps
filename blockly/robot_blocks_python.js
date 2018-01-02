// Python code generation

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

Blockly.Python['get_pose'] = function(block) {
  var code = 'get_robot_pose()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['obstacle_distance'] = function(block) {
  var code = 'obstacle_distance()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['distance'] = function(block) {
  var value_p1 = Blockly.Python.valueToCode(block, 'P1', Blockly.Python.ORDER_ATOMIC);
  var value_p2 = Blockly.Python.valueToCode(block, 'P2', Blockly.Python.ORDER_ATOMIC);
  var code = 'distance('+value_p1+','+value_p2+')';
  return [code, Blockly.Python.ORDER_NONE];
};
