export function load(Blockly) {
  Blockly.Blocks["dt_get_stop_line_duckietown"] = {
    init: function () {
      this.jsonInit({
        type: "block_type",
        message0: "%{BKY_SET_ANALOG_PIN}",
        args0: [
          {
            type: "field_input",
            name: "PIN",
            text: "A0",
          },
          {
            type: "input_value",
            name: "VALUE",
            check: "Number",
          },
        ],
        inputsInline: true,
        previousStatement: null,
        nextStatement: null,
        colour: "%{BKY_ACTIONS_RGB}",
      });
    },
  };

  Blockly.Python["dt_get_stop_line_duckietown"] = function (block) {
    Blockly.Python.definitions_["import_dtcv"] =
      "from cv import Camera\nprocecoort= Camera()";
    return `processor.getStopLine()\n`;
  };
}
