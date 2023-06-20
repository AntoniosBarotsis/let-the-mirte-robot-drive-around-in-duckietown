<template>
  
     <div class="layoutbox-content">
        <div v-for="sensor in getSensorTypes()"  class="rounded background-green-light p-3 mb-2">
              <h5>{{ $t('peripherals.' + peripherals[sensor].text) }}</h5>
              <div class="row">
 
                <div class="col-4">
                  <img class="center-div w-75" :src="getSensorImage(sensor)">
                </div>

                <div class="col-8">

                  <div v-for="instance in getInstancesOfSensor(sensor)" class="rounded background-primary p-2 text-white mb-2">
                     {{instance}}: {{ sensor_values[sensor][instance] }}
                  </div>
                </div>
              </div>
        </div>
    </div>

</template>

<script>
import properties_ph from "../assets/json/properties_ph.json"
import MenuButtons from '@/components/MenuButtons.vue'
import ROSLIB from 'roslib'
import ros from '../ws-connection/ROS-connection.js'
import Vue from 'vue'

export default {
  name: 'sensors',
  components: {
    MenuButtons
  },
  data() {
    return {
      peripherals: properties_ph,
      param_sensors: {},
      sensor_values: {}
    }
  },
  methods: {

    //separates peripheral items into sensors and actuators
    getSensorTypes() {
      let types = [];
      for (let type in this.param_sensors){
         types.push(type);
      }
      return types;
    },
    // Load the configuarion from ROS params to Vue object items
    loadConfiguration(rosparams){
      let sensors_copy = Object.assign({}, rosparams);
      for (let type in rosparams){
         for (let instance in rosparams[type]){
             if (!this.peripherals.hasOwnProperty(type) || this.peripherals[type].rel_path.split("\\")[0] !== "Sensors" ){
                 delete sensors_copy[type];
             }
         }
      }
      this.param_sensors = sensors_copy;
    },
    getInstancesOfSensor(type){
       let ret = [];
       for (let sensor_instance in this.param_sensors[type]){
           ret.push(sensor_instance);
       }
       return ret;
    },
    getSensorImage(type){
      var images = require.context('../assets/images/', false, /\.jpg$/)
      return images('./' + type + ".jpg")
    }
  },
  mounted(){
     // TODO: loading paramaters twice. This should not be needed
     let params = new ROSLIB.Param({
       ros: ros,
       name: '/mirte'
     })

     params.get((res) => {
            this.loadConfiguration(res);
            const sensors = this.getSensorTypes();
            for (let sensor in sensors){

               Vue.set(this.sensor_values, sensors[sensor], {});
               const instances = this.getInstancesOfSensor(sensors[sensor]);
               for (let instance in instances){

                  Vue.set(this.sensor_values[sensors[sensor]], instances[instance], -1);

                  let topic = new ROSLIB.Topic({
                     ros : ros,
                     name : '/mirte/' + sensors[sensor] + '/' + instances[instance],
                     messageType : this.peripherals[sensors[sensor]].message_type
                  });
                  topic.subscribe((message) => {
                     this.sensor_values[sensors[sensor]][instances[instance]] = message[this.peripherals[sensors[sensor]].message_value];
                  });
               }
            }
        });
  }
}
</script>
