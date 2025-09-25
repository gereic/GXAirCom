const SETTING_BASIC = 0;
const SETTING_ADVANCED = 100;
const SETTING_EXPERT = 200;

const AIR_MODULE = 0;
const GROUND_STATION = 1;

const CONNECT_NONE = 0;
const CONNECT_ONCE = 1;
const CONNECT_ALWAYS = 2;

const DAVIS = 0;
const TX20 = 1;
const ADS_A1015 = 2;
const PEETBROS = 3;
const MISOL = 4;
const WS90 = 5;
const WS85_serial = 6;

const serial = 0;
const udp = 1;
const ble = 3;

const GS_POWER_ALWAYS_ON = 0;
const GS_POWER_SAFE = 1;
const GS_POWER_BATT_LIFE = 2;

const NO_DISPLAY = 0;


const url = `ws://${window.location.hostname}:1337/`; /* we have to split the url, otherwise minify detects it as comment */
let websocket;

/* Called when the page finishes loading */
function init() {
  wsConnect(url);
}

/* Connect to the WebSocket server */
function wsConnect(url) {
  console.log(`Trying to connect to ${url}`);
  websocket = new WebSocket(url);

  websocket.onopen = onOpen;
  websocket.onclose = onClose;
  websocket.onmessage = onMessage;
  websocket.onerror = onError;
}

/* WebSocket connected */
function onOpen(evt) {
  console.log("Connected");
  if (typeof pageNumber !== "undefined") {
    doSend(JSON.stringify({ page: pageNumber }));
  } else {
    console.warn("pageNumber is not defined; skipping initial send.");
  }
}

/* WebSocket closed */
function onClose(evt) {
  console.log("Disconnected");
  setTimeout(() => wsConnect(url), 2000);
}

function callMainPage() {
  window.location = "/index.html";
}

function setElement(name,value){
    var e = document.getElementById(name);
    if (e == null) return;
    if (e instanceof HTMLSelectElement) {     /* <select>*/
      e.value = value;
    }else if ((e instanceof HTMLInputElement ) && (e.getAttribute('type') == 'checkbox')){     /* <input checkbox>*/
      if (value == 1){
        e.checked = true;
      }else{
        e.checked = false;
      }
    }else{
      e.textContent = value;
      e.value = value;
    }
}

function FntIdDec2Hex(value) {
  let retVal = "";
  if (value !== 0) {
    retVal = value.toString(16).toUpperCase().padStart(2, "0");
    while (retVal.length < 6) {
      retVal = "0" + retVal;
    }
  }
  return retVal;
}

function FntIdHex2Dec(value) {
  const retVal = parseInt(value, 16);
  return isNaN(retVal) ? 0 : retVal;
}

function onError(evt) {
  console.log("ERROR:", evt);
}

function doSend(message) {
  console.log("Sending:", message);
  if (websocket && websocket.readyState === WebSocket.OPEN) {
    websocket.send(message);
  } else {
    console.warn("WebSocket not open; message not sent.");
  }
}

window.addEventListener("load", init, false);

function getValue(obj,elements){
  elements.forEach(element => {
    var e = document.getElementById(element);
    //console.log("type=" + e.tagName);
    if (e.tagName == 'select-one'){
      //console.log("select " + element);
      obj[element] = Number(e.checked);
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

function setvisible(element_id,bValue){
  document.getElementById(element_id).style.display= (bValue == true) ? '': 'none';
}
 