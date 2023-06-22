export function load(Blockly) {
  Blockly.Blocks["sees_stop_line_duckietown"] = {
    init: function() {
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
    init: function() {
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
    init: function() {
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

  Blockly.Blocks["sees_sign_duckietown"] = {
    init: function() {
      this.jsonInit({
        type: "block_type",
        message0: "%{BKY_SIGN}",
        args0: [
          {
            type: "field_dropdown",
            name: "SIGN",
            options: [
              ["%{BKY_STOP}", "Sign.STOP"],
              ["%{BKY_YIELD}", "Sign.YIELD"],
              ["%{BKY_NO_RIGHT_TURN}", "Sign.NO_RIGHT_TURN"],
              ["%{BKY_NO_LEFT_TURN}", "Sign.NO_LEFT_TURN"],
              ["%{BKY_DO_NOT_ENTER}", "Sign.DO_NOT_ENTER"],
              ["%{BKY_ONEWAY_RIGHT}", "Sign.ONEWAY_RIGHT"],
              ["%{BKY_ONEWAY_LEFT}", "Sign.ONEWAY_LEFT"],
              ["%{BKY_FOUR_WAY_INTERSECT}", "Sign.FOUR_WAY_INTERSECT"],
              ["%{BKY_RIGHT_T_INTERSECT}", "Sign.RIGHT_T_INTERSECT"],
              ["%{BKY_LEFT_T_INTERSECT}", "Sign.LEFT_T_INTERSECT"],
              ["%{BKY_T_INTERSECTION}", "Sign.T_INTERSECTION"],
              ["%{BKY_PEDESTRIAN}", "Sign.PEDESTRIAN"],
              ["%{BKY_T_LIGHT_AHEAD}", "Sign.T_LIGHT_AHEAD"],
              ["%{BKY_DUCK_CROSSING}", "Sign.DUCK_CROSSING"],
              ["%{BKY_PARKING}", "Sign.PARKING"],
              ["%{BKY_STREET}", "Sign.STREET"],
            ],
          },
        ],
        output: "Boolean",
        colour: "%{BKY_DUCKIE_RGB}",
        inputsInline: true,
        tooltip: "%{BKY_SIGN_TIP}",
      });
    },
  };

  Blockly.Blocks["sees_street_duckietown"] = {
    init: function() {
      this.jsonInit({
        type: "block_type",
        message0: "%{BKY_STREET_SIGN}",
        args0: [
          {
            type: "field_input",
            name: "STREET",
            text: "DUDEK ST",
          },
        ],
        output: "Boolean",
        colour: "%{BKY_DUCKIE_RGB}",
        inputsInline: true,
        tooltip: "%{BKY_STREET_SIGN_TIP}",
      });
    },
  };

  Blockly.Python["sees_stop_line_duckietown"] = function(block) {
    Blockly.Python.definitions_["import_mirte"] =
      "from mirte_robot import robot\nmirte=robot.createRobot()";
    Blockly.Python.definitions_["import_duckietown"] =
      "from mirte_duckietown import duckietown\ncamera=duckietown.createCamera(mirte)";
    let code = `camera.seesStopLine()`;
    return [code, Blockly.Python.ORDER_NONE];
  };

  Blockly.Python["start_following_duckietown"] = function(block) {
    Blockly.Python.definitions_["import_mirte"] =
      "from mirte_robot import robot\nmirte=robot.createRobot()";
    Blockly.Python.definitions_["import_duckietown"] =
      "from mirte_duckietown import duckietown\ncamera=duckietown.createCamera(mirte)";
    let code = `camera.startFollowing()\n`;
    return code;
  };

  Blockly.Python["stop_following_duckietown"] = function(block) {
    Blockly.Python.definitions_["import_mirte"] =
      "from mirte_robot import robot\nmirte=robot.createRobot()";
    Blockly.Python.definitions_["import_duckietown"] =
      "from mirte_duckietown import duckietown\ncamera=duckietown.createCamera(mirte)";
    let code = `camera.stopFollowing()\n`;
    return code;
  };

  Blockly.Python["sees_sign_duckietown"] = function(block) {
    Blockly.Python.definitions_["import_mirte"] =
      "from mirte_robot import robot\nmirte=robot.createRobot()";
    Blockly.Python.definitions_["import_duckietown"] =
      "from mirte_duckietown import duckietown\ncamera=duckietown.createCamera(mirte)";
    Blockly.Python.definitions_["import_sign"] = "from sign import Sign";
    let code = `camera.seesSign(${block.getFieldValue("SIGN")})`;
    return [code, Blockly.Python.ORDER_NONE];
  };

  Blockly.Python["sees_street_duckietown"] = function(block) {
    Blockly.Python.definitions_["import_mirte"] =
      "from mirte_robot import robot\nmirte=robot.createRobot()";
    Blockly.Python.definitions_["import_duckietown"] =
      "from mirte_duckietown import duckietown\ncamera=duckietown.createCamera(mirte)";
    let code = `camera.seesStreet("${block.getFieldValue("STREET")}")`;
    return [code, Blockly.Python.ORDER_NONE];
  };
}
