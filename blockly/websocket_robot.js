// log display function
function append(text) {
  // document.getElementById("websocket_events").insertAdjacentHTML('beforeend', "<li>" + text + ";</li>");
  // log.console(text);
} 

// websocket global variable
var websocket = null;


function wsrobot_init() {
    var ip = document.getElementById("IP").value
    var url = "ws://"+ip+":9000/websocketserver"
    console.log(url)
    websocket = new WebSocket(url);
}
 
window.onload = function(){

    websocket.onmessage = function(event){
      append("message received: "+event.data) 
    } 

    websocket.onopen = function(){
      append("connection received") 
    } 

    websocket.onclose = function(){
      append("connection closed");
    }

    websocket.onerror = function(){
      append("!!!connection error!!!");
    }

}

function wsrobot_send(data) {
  websocket.send(data);
}


function button_fn(data) {
  // console.log('websocket button...')
  wsrobot_send(data);
}


