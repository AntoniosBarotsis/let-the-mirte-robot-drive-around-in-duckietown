export function load(Blockly) {
  Blockly.Blocks["dt_stop_line"] = {
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

  Blockly.Python["dt_stop_line"] = function (block) {
    Blockly.Python.definitions_["import_duckietown"] =
      "from duckietown import Camera\ncamera = Camera()";
    let code = `camera.stopLine()`;
    return [code, Blockly.Python.ORDER_NONE]
  };
}
