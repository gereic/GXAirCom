const SETTING_BASIC = 0;
const SETTING_ADVANCED = 100;
const SETTING_EXPERT = 200;

var url = "ws://" + window.location.hostname + ":1337/";

// This is called when the page finishes loading
function init() {
 
  // Connect to WebSocket server
  wsConnect(url);
}

// Call this to connect to the WebSocket server
function wsConnect(url) {
  
  // Connect to WebSocket server
  //console.log("try to connect to " + url);    
  //document.getElementById("TxtConn").innerHTML ="try to connect to " + url;
  websocket = new WebSocket(url);
  
  // Assign callbacks
  websocket.onopen = function(evt) { onOpen(evt) };
  websocket.onclose = function(evt) { onClose(evt) };
  websocket.onmessage = function(evt) { onMessage(evt) };
  websocket.onerror = function(evt) { onError(evt) };
}

// Called when a WebSocket connection is established with the server
function onOpen(evt) {
 
  // Log connection state
  console.log("Connected");
  //document.getElementById("TxtConn").innerHTML ="connected";
  // write page-number --> then we get all values for page
  doSend(JSON.stringify({ page : pageNumber }));//send page-number
}

// Called when the WebSocket connection is closed
function onClose(evt) {

  // Log disconnection state
  console.log("Disconnected");
  //document.getElementById("TxtConn").innerHTML ="disconnected";
  // Try to reconnect after a few seconds
  setTimeout(function() { wsConnect(url) }, 2000);
}

function callMainPage(){
  window.location="/index.html"
}

function FntIdDec2Hex(value){
  var retVal = "";
  if (value != 0){
    retVal = (value).toString(16).toUpperCase().padStart(2, '0')
    console.log("value="+value+",l="+retVal.length+"retval=" + retVal);
    while (retVal.length < 6){
      retVal = "0" + retVal;
    };
  }
  return retVal;
}
 
function FntIdHex2Dec(value){
  var retVal = parseInt(value,16);
  if (!retVal){
    retVal = 0;
  }
  //console.log("value="+value+"retval=" + retVal);
  return retVal;
}
 
// Called when a WebSocket error occurs
function onError(evt) {
  console.log("ERROR: " + evt.data);
  //document.getElementById("TxtConn").innerHTML ="error: " + evt.data;
}

// Sends a message to the server (and prints it to the console)
function doSend(message) {
  console.log("Sending: " + message);
  websocket.send(message);
}

// Call the init function as soon as the page loads
window.addEventListener("load", init, false);

function getValue(obj,elements){
  elements.forEach(element => {
    var e = document.getElementById(element);
    //console.log("type=" + e.tagName);
    if (e.tagName == 'select-one'){
      //console.log("select " + element);
      obj[element] = NUMBER(e.checked);
    }else if (e.getAttribute('type') == 'checkbox') {
      //console.log("checkbox " + element);  
      obj[element] = Number(e.checked);
    } else if (e.getAttribute('type') == 'number'){
      //console.log("number " + element);  
      obj[element] = Number(e.value);
    } else {
      //console.log("string " + element);  
      obj[element] = e.value;
    }
  });
  return obj;
}



function doSendData(elements, save) {
  var obj = { save: save };
  getValue(obj,elements);
  doSend(JSON.stringify(obj));
}


function noType(element) {
    return element;
}

//get value of element
function _value(element_id) {
  return document.getElementById(element_id).value;
}
 
