import ROSLIB from 'roslib'

const ros_protocol = (location.protocol === 'https:') ? 'wss://' : 'ws://';
const ros_socketUrl = `${ros_protocol}${location.hostname}/ws/ros`;

const ros = new ROSLIB.Ros({
    url : ros_socketUrl
});

export default ros;
