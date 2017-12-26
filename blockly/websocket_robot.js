append = function(text){
  document.getElementById("websocket_events").insertAdjacentHTML('beforeend', 
                          "<li>" + text + ";</li>");
} 

var websocket = new WebSocket("ws://127.0.0.1:9000/websocketserver");

window.onload = function(){

    websocket.onmessage = function(event){
      append("message received: "+event.data) 
    } 

    websocket.onopen = function(){
      append("connection received") 
      websocket.send("hello!");
    } 

    websocket.onclose = function(){
      append("connection closed");
    }

    websocket.onerror = function(){
      append("!!!connection error!!!");
    }

}

function ws_send(data) {
  websocket.send(data);
}


function button_fn(data) {
  // console.log('websocket button...')
  websocket.send(data);
}


