const protocol = (location.protocol === 'https:') ? 'wss://' : 'ws://';
const shell_socketUrl = `${protocol}${location.hostname}/ws/shell`;
const shell_socket = new WebSocket(shell_socketUrl);

export default shell_socket;
