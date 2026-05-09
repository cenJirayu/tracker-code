// serial.js — Web Serial + UI handlers (non-module, works from file://)
var port=null,reader=null,writer=null,readableStreamClosed=null,isConnected=false,partialLine='';
var chartData={azT:[],azC:[],elT:[],elC:[],ts:[]};
var MAX_CHART=300;
var _3dUpdate=null, _3dTrackerUpdate=null; // hooks for Three.js (set by module script)

function logSerial(msg,color){
    color=color||'#94a3b8';
    var el=document.getElementById('serialLog');
    var d=document.createElement('div');
    d.style.color=color;
    d.textContent=msg;
    el.appendChild(d);
    if(el.children.length>200)el.removeChild(el.firstChild);
    el.scrollTop=el.scrollHeight;
}

async function sendString(s){
    if(!writer){logSerial('Not connected','#f87171');return;}
    try{await writer.write(new TextEncoder().encode(s+'\n'));}
    catch(e){logSerial('Write error: '+e.message,'#f87171');}
}

function sendCmd(o){
    var s=JSON.stringify(o);
    sendString(s);
    logSerial('> '+s,'#6c8cff');
}

function processIncoming(line){
    logSerial('< '+line,'#94a3b8');
    try{
        var d=JSON.parse(line);
        if(d.t==='tel'){
            document.getElementById('telAzT').textContent=d.az_t+'°';
            document.getElementById('telAzC').textContent=d.az_c+'°';
            document.getElementById('telElT').textContent=d.el_t+'°';
            document.getElementById('telElC').textContent=d.el_c+'°';
            document.getElementById('telPid').textContent=d.pid;
            document.getElementById('telSpd').textContent=d.spd;
            document.getElementById('telSweep').textContent=d.sweep||'none';
            document.getElementById('telUp').textContent=(d.up/1000).toFixed(1)+'s';
            if(_3dTrackerUpdate)_3dTrackerUpdate(parseFloat(d.az_c),parseFloat(d.el_c));
            var now=performance.now()/1000;
            chartData.azT.push(parseFloat(d.az_t));chartData.azC.push(parseFloat(d.az_c));
            chartData.elT.push(parseFloat(d.el_t));chartData.elC.push(parseFloat(d.el_c));
            chartData.ts.push(now);
            if(chartData.ts.length>MAX_CHART){chartData.azT.shift();chartData.azC.shift();chartData.elT.shift();chartData.elC.shift();chartData.ts.shift();}
            drawChart();
        }
        if(d.t==='ready'){logSerial('ESP32 ready: motor='+d.motor+' servo='+d.servo,'#34d399');}
        if(d.t==='ack'){logSerial('ACK: '+d.cmd,'#34d399');}
        if(d.t==='err'){logSerial('ERR: '+d.msg,'#f87171');}
    }catch(e){}
}

function updateConnectionUI(c){
    document.getElementById('statusRow').className='status-row '+(c?'connected':'disconnected');
    document.getElementById('statusText').textContent=c?'Connected':'Disconnected';
    var b=document.getElementById('btnConnect');
    b.textContent=c?'Disconnect':'Connect';
    b.className=c?'btn btn-red':'btn btn-green';
}

async function readLoop(){
    var decoder=new TextDecoderStream();
    readableStreamClosed=port.readable.pipeTo(decoder.writable);
    reader=decoder.readable.getReader();
    try{
        while(true){
            var r=await reader.read();
            if(r.done)break;
            partialLine+=r.value;
            var lines=partialLine.split('\n');
            partialLine=lines.pop();
            for(var i=0;i<lines.length;i++){
                var t=lines[i].trim();
                if(t)processIncoming(t);
            }
        }
    }catch(e){if(isConnected)logSerial('Read error: '+e.message,'#f87171');}
}

async function toggleConnection(){
    if(isConnected){
        isConnected=false;
        try{
            if(reader){await reader.cancel();reader.releaseLock();}
            if(readableStreamClosed)await readableStreamClosed.catch(function(){});
            if(writer)writer.releaseLock();
            if(port)await port.close();
        }catch(e){}
        reader=null;writer=null;port=null;
        updateConnectionUI(false);
        logSerial('--- Disconnected ---','#f87171');
        return;
    }
    if(!navigator.serial){
        logSerial('ERROR: Web Serial API not supported. Use Chrome or Edge.','#f87171');
        alert('Web Serial API not supported!\nPlease use Chrome or Edge browser.');
        return;
    }
    try{
        port=await navigator.serial.requestPort();
        await port.open({baudRate:115200});
        writer=port.writable.getWriter();
        isConnected=true;
        updateConnectionUI(true);
        readLoop();
        logSerial('--- Connected ---','#34d399');
    }catch(e){logSerial('Connection failed: '+e.message,'#f87171');}
}

function sendSensorConfig(){
    sendCmd({cmd:'set_sensors',enc:document.getElementById('chkEncoder').checked,mag:document.getElementById('chkMag').checked,gps:document.getElementById('chkGPS').checked});
}

var _targetAz=0,_targetEl=0;

function onElSlider(v){
    v=parseFloat(v);
    document.getElementById('elVal').textContent=v.toFixed(1)+'°';
    sendCmd({cmd:'direct',el:v});
    _targetEl=v;
    if(_3dUpdate)_3dUpdate(_targetAz,_targetEl);
}
function onAzSlider(v){
    v=parseFloat(v);
    document.getElementById('azVal').textContent=v.toFixed(1)+'°';
    sendCmd({cmd:'direct',az:v});
    _targetAz=v;
    if(_3dUpdate)_3dUpdate(_targetAz,_targetEl);
}
function directEl(v){document.getElementById('elSlider').value=v;onElSlider(v);}
function directAz(v){document.getElementById('azSlider').value=v;onAzSlider(v);}

function onPidChange(){
    var kp=parseFloat(document.getElementById('kpSlider').value);
    var ki=parseFloat(document.getElementById('kiSlider').value);
    var kd=parseFloat(document.getElementById('kdSlider').value);
    document.getElementById('kpVal').textContent=kp.toFixed(2);
    document.getElementById('kiVal').textContent=ki.toFixed(3);
    document.getElementById('kdVal').textContent=kd.toFixed(2);
    sendCmd({cmd:'set_pid',kp:kp,ki:ki,kd:kd});
}

function sendBase(){
    sendCmd({cmd:'set_base',lat:parseFloat(document.getElementById('baseLat').value),lon:parseFloat(document.getElementById('baseLon').value),alt:parseFloat(document.getElementById('baseAlt').value)});
}
function injectCoords(){
    sendCmd({cmd:'inject',lat:parseFloat(document.getElementById('tgtLat').value),lon:parseFloat(document.getElementById('tgtLon').value),alt:parseFloat(document.getElementById('tgtAlt').value)});
}
function preset(p){
    var bLat=parseFloat(document.getElementById('baseLat').value);
    var bLon=parseFloat(document.getElementById('baseLon').value);
    var presets={n1k:{lat:bLat+0.009,lon:bLon,alt:100},e1k:{lat:bLat,lon:bLon+0.0105,alt:100},s1k:{lat:bLat-0.009,lon:bLon,alt:100},w1k:{lat:bLat,lon:bLon-0.0105,alt:100},overhead:{lat:bLat,lon:bLon,alt:1000}};
    var t=presets[p];if(!t)return;
    document.getElementById('tgtLat').value=t.lat.toFixed(4);
    document.getElementById('tgtLon').value=t.lon.toFixed(4);
    document.getElementById('tgtAlt').value=t.alt;
    injectCoords();
}

function drawChart(){
    var canvas=document.getElementById('chartCanvas');
    var rect=canvas.getBoundingClientRect();
    canvas.width=rect.width*devicePixelRatio;
    canvas.height=rect.height*devicePixelRatio;
    var ctx=canvas.getContext('2d');
    ctx.scale(devicePixelRatio,devicePixelRatio);
    var W=rect.width,H=rect.height;
    ctx.clearRect(0,0,W,H);
    if(chartData.ts.length<2)return;
    ctx.strokeStyle='rgba(100,120,200,0.1)';ctx.lineWidth=1;
    for(var i=0;i<5;i++){var y=H*i/4;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}
    ctx.fillStyle='#64748b';ctx.font='10px Inter';
    ctx.fillText('360°',2,12);ctx.fillText('180°',2,H/2+4);ctx.fillText('0°',2,H-2);
    var tMin=chartData.ts[0],tMax=chartData.ts[chartData.ts.length-1],tRange=Math.max(tMax-tMin,1);
    function drawLine(data,color,maxV){
        maxV=maxV||360;ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=1.5;
        for(var i=0;i<data.length;i++){var x=(chartData.ts[i]-tMin)/tRange*W,yy=H-(data[i]/maxV)*H;i===0?ctx.moveTo(x,yy):ctx.lineTo(x,yy);}
        ctx.stroke();
    }
    drawLine(chartData.azT,'rgba(108,140,255,0.5)');drawLine(chartData.azC,'#6c8cff');
    drawLine(chartData.elT,'rgba(167,139,250,0.5)',180);drawLine(chartData.elC,'#a78bfa',180);
    ctx.font='10px Inter';
    var labels=[{c:'#6c8cff',l:'Az Cur'},{c:'rgba(108,140,255,0.5)',l:'Az Tgt'},{c:'#a78bfa',l:'El Cur'},{c:'rgba(167,139,250,0.5)',l:'El Tgt'}];
    var lx=W-200;
    for(var j=0;j<labels.length;j++){ctx.fillStyle=labels[j].c;ctx.fillRect(lx,4,10,8);ctx.fillStyle='#94a3b8';ctx.fillText(labels[j].l,lx+14,12);lx+=50;}
}

function sendManualCmd(){
    var inp=document.getElementById('manualCmd');
    var s=inp.value.trim();
    if(!s)return;
    try{JSON.parse(s);sendString(s);logSerial('> '+s,'#6c8cff');}
    catch(e){logSerial('Invalid JSON: '+e.message,'#f87171');}
    inp.value='';
}

// Startup check
document.addEventListener('DOMContentLoaded',function(){
    if(!navigator.serial){
        logSerial('⚠ Web Serial API not available. Use Chrome or Edge.','#fbbf24');
    } else {
        logSerial('✓ Web Serial API available. Click Connect to start.','#34d399');
    }
});
