const SETTING_BASIC = 0;
const SETTING_ADVANCED = 100;
const SETTING_EXPERT = 200;

const url = "ws:/" + `/${window.location.hostname}:1337/`; /* we have to split the url, otherwise minify detects it as comment */
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