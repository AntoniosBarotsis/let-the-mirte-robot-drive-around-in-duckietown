export function load(Blockly) {
  Blockly.Blocks["sees_stop_line_duckietown"] = {
    init: function () {
      this.jsonInit({
        type: "block_type",
        message0: "%{BKY_STOP_LINE}",
        args0: [],
        output: "Boolean",
        colour: "%{BKY_DUCKIE_RGB}",
        inputsInline: true,
        tooltip: "%{BKY_STOP_LINE_TIP}",
      });
    },
  };

  Blockly.Blocks["start_following_duckietown"] = {
    init: function () {
      this.jsonInit({
        type: "block_type",
        message0: "%{BKY_START_FOLLOWING}",
        args0: [],
        previousStatement: null,
        nextStatement: null,
        colour: "%{BKY_DUCKIE_RGB}",
        inputsInline: true,
        tooltip: "%{BKY_START_FOLLOWING_TIP}",
      });
    },
  };

  Blockly.Blocks["stop_following_duckietown"] = {
    init: function () {
      this.jsonInit({
        type: "block_type",
        message0: "%{BKY_STOP_FOLLOWING}",
        args0: [],
        previousStatement: null,
        nextStatement: null,
        colour: "%{BKY_DUCKIE_RGB}",
        inputsInline: true,
        tooltip: "%{BKY_STOP_FOLLOWING_TIP}",
      });
    },
  };

  
  Blockly.Python["sees_stop_line_duckietown"] = function (block) {
    Blockly.Python.definitions_['import_mirte'] = 'from mirte_robot import robot\nmirte=robot.createRobot()';
    Blockly.Python.definitions_["import_duckietown"] =
      "from mirte_duckietown import duckietown\ncamera=duckietown.createCamera(mirte)";
    let code = `camera.seesStopLine()`;
    return [code, Blockly.Python.ORDER_NONE];
  };

  Blockly.Python["start_following_duckietown"] = function (block) {
    Blockly.Python.definitions_['import_mirte'] = 'from mirte_robot import robot\nmirte=robot.createRobot()';
    Blockly.Python.definitions_["import_duckietown"] =
      "from mirte_duckietown import duckietown\ncamera=duckietown.createCamera(mirte)";
    let code = `camera.startFollowing()\n`;
    return code;
  };

  Blockly.Python["stop_following_duckietown"] = function (block) {
    Blockly.Python.definitions_['import_mirte'] = 'from mirte_robot import robot\nmirte=robot.createRobot()';
    Blockly.Python.definitions_["import_duckietown"] =
      "from mirte_duckietown import duckietown\ncamera=duckietown.createCamera(mirte)";
    let code = `camera.stopFollowing()\n`;
    return code;
  };
}
