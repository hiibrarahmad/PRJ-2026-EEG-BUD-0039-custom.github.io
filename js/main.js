
/*******************************************************************************************************************
 *********************************************** WEB BLUETOOTH *****************************************************
 *******************************************************************************************************************/

//sensor data object
var state = {};

var dataType = "FIR";
var secondDataSet = false;
// Web Bluetooth connection -->
$( document ).ready(function() {
    button = document.getElementById("connect");
    message = document.getElementById("message");
});

var _this;

var sendCommandFlag = false; //global to keep track of when command is sent back to device
//let commandValue = new Uint8Array([0x01,0x03,0x02,0x03,0x01]);   //command to send back to device
let commandValue = new Uint8Array([0x99]); //command to send back to device
//connection flag
var bluetoothDataFlag = false;

if ( 'bluetooth' in navigator === false ) {
    button.style.display = 'none';
    message.innerHTML = 'This browser doesn\'t support the <a href="https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API" target="_blank">Web Bluetooth API</a> :(';
}

const services = {
    controlService: {
        name: 'control service',
        uuid: '0000a000-0000-1000-8000-00805f9b34fb'
    }
}

const characteristics = {
    commandReadCharacteristic: {
        name: 'command read characteristic',
        uuid: '0000a001-0000-1000-8000-00805f9b34fb'
    },
    commandWriteCharacteristic: {
        name: 'command write characteristic',
        uuid: '0000a002-0000-1000-8000-00805f9b34fb'
    },
    deviceDataCharacteristic: {
        name: 'eeg data characteristic',
        uuid: '0000a003-0000-1000-8000-00805f9b34fb'
    }
}



class ControllerWebBluetooth {
    constructor(name) {
        _this = this;
        this.name = name;
        this.services = services;
        this.characteristics = characteristics;
        this.standardServer;
    }

    connect() {
        return navigator.bluetooth.requestDevice({
            filters: [{
                        name: this.name
                    },
                    {
                        services: [services.controlService.uuid]
                    }
                ]
            })
            .then(device => {
                console.log('Device discovered', device.name);
                return device.gatt.connect();
            })
            .then(server => {
                console.log('server device: ' + Object.keys(server.device));

                this.getServices([services.controlService, ], [characteristics.commandReadCharacteristic, characteristics.commandWriteCharacteristic, characteristics.deviceDataCharacteristic], server);
            })
            .catch(error => {
                console.log('error', error)
            })
    }

    getServices(requestedServices, requestedCharacteristics, server) {
        this.standardServer = server;

        requestedServices.filter((service) => {
            if (service.uuid == services.controlService.uuid) {
                _this.getControlService(requestedServices, requestedCharacteristics, this.standardServer);
            }
        })
    }

    getControlService(requestedServices, requestedCharacteristics, server) {
        let controlService = requestedServices.filter((service) => {
            return service.uuid == services.controlService.uuid
        });
        let commandReadChar = requestedCharacteristics.filter((char) => {
            return char.uuid == characteristics.commandReadCharacteristic.uuid
        });
        let commandWriteChar = requestedCharacteristics.filter((char) => {
            return char.uuid == characteristics.commandWriteCharacteristic.uuid
        });

        // Before having access to eeg, EMG and Pose data, we need to indicate to the Myo that we want to receive this data.
        return server.getPrimaryService(controlService[0].uuid)
            .then(service => {
                console.log('getting service: ', controlService[0].name);
                return service.getCharacteristic(commandWriteChar[0].uuid);
            })
            .then(characteristic => {
                console.log('getting characteristic: ', commandWriteChar[0].name);
                // return new Buffer([0x01,3,emg_mode,eeg_mode,classifier_mode]);
                // The values passed in the buffer indicate that we want to receive all data without restriction;
                //  let commandValue = new Uint8Array([0x01,0x03,0x02,0x03,0x01]);
                //this could be config info to be sent to the wearable device
                let commandValue = new Uint8Array([0x99]);
                //   characteristic.writeValue(commandValue); //disable initial write to device
            })
            .then(_ => {

                let deviceDataChar = requestedCharacteristics.filter((char) => {
                    return char.uuid == characteristics.deviceDataCharacteristic.uuid
                });

                console.log('getting service: ', controlService[0].name);
                _this.getdeviceData(controlService[0], deviceDataChar[0], server);

            })
            .catch(error => {
                console.log('error: ', error);
            })
    }

    sendControlService(requestedServices, requestedCharacteristics, server) {
        let controlService = requestedServices.filter((service) => {
            return service.uuid == services.controlService.uuid
        });
        let commandReadChar = requestedCharacteristics.filter((char) => {
            return char.uuid == characteristics.commandReadCharacteristic.uuid
        });
        let commandWriteChar = requestedCharacteristics.filter((char) => {
            return char.uuid == characteristics.commandWriteCharacteristic.uuid
        });

        return server.getPrimaryService(controlService[0].uuid)
            .then(service => {
                console.log('getting service: ', controlService[0].name);
                return service.getCharacteristic(commandWriteChar[0].uuid);
            })
            .then(characteristic => {
                console.log('getting write command to device characteristic: ', commandWriteChar[0].name);
                // return new Buffer([0x01,3,emg_mode,eeg_mode,classifier_mode]);
                // The values passed in the buffer indicate that we want to receive all data without restriction;
                let commandValue = new Uint8Array([0x99]);
                getConfig();
                commandValue[0] = targetCommand;

                console.log("CONFIG target:" + activeTarget + "  command:" + commandValue[0]);
                characteristic.writeValue(commandValue);
            })
            .then(_ => {

                //  let deviceDataChar = requestedCharacteristics.filter((char) => {return char.uuid == characteristics.deviceDataCharacteristic.uuid});
                console.log("COMMAND SENT TO DEVICE");
                sendCommandFlag = false;
                //   console.log('getting service: ', controlService[0].name);
                //  _this.getdeviceData(controlService[0], deviceDataChar[0], server);

            })
            .catch(error => {
                sendCommandFlag = false;
                console.log("COMMAND SEND ERROR");
                console.log('error: ', error);
            })
    }


    handleDeviceDataChanged(event) {
        // ── New packet format (firmware xiao_nrf52840_eeg) ──────────────────
        // Each sample = 27 bytes:
        //   [0xA0][seq][ch1_b2][ch1_b1][ch1_b0] … [ch8_b2][ch8_b1][ch8_b0][0xC0]
        // Multiple samples may be batched in one BLE notification (MTU-sized).
        // 24-bit values are two's complement, MSB first, from ADS1299 at Gain=24.
        // ────────────────────────────────────────────────────────────────────

        const SAMPLE_SIZE  = 27;
        const START_BYTE   = 0xA0;
        const END_BYTE     = 0xC0;
        const NORM_SCALE   = 16777215; // 2^24 - 1

        const bytes = new Uint8Array(event.target.value.buffer);
        const samples = [];

        // Parse every complete sample frame in the notification
        for (let i = 0; i <= bytes.length - SAMPLE_SIZE; i++) {
            if (bytes[i] === START_BYTE && bytes[i + SAMPLE_SIZE - 1] === END_BYTE) {
                const channels = [];
                for (let ch = 0; ch < 8; ch++) {
                    const off = i + 2 + ch * 3;
                    // Reconstruct 24-bit two's-complement signed integer
                    let raw = (bytes[off] << 16) | (bytes[off + 1] << 8) | bytes[off + 2];
                    if (raw & 0x800000) raw = raw - 0x1000000; // sign-extend
                    channels.push(raw);
                }
                samples.push(channels);
                i += SAMPLE_SIZE - 1; // advance past this frame
            }
        }

        if (samples.length === 0) return; // guard: no valid frames found

        // Use the most recent complete sample for display
        const ch = samples[samples.length - 1];

        // Normalise each 24-bit signed value to 0..1  (0.5 = zero ADC)
        // This keeps the existing NN and chart scaling code working unchanged.
        const norm = raw => (raw + 8388608) / NORM_SCALE;

        const n = ch.map(norm); // n[0]..n[7] = ch1..ch8 normalised

        // Build µV values for logging (ADS1299: Vref=4.5V, Gain=24)
        const LSB_UV = (4.5 / 24.0 / 8388608.0) * 1e6; // ≈ 0.02235 µV/LSB
        const uv = ch.map(raw => (raw * LSB_UV).toFixed(3));
        console.log("[EEG µV] ch1-8: " + uv.join(", ") + "  samples/pkt: " + samples.length);

        state = {
            // Raw channel access (normalised 0-1)
            ch1: n[0], ch2: n[1], ch3: n[2], ch4: n[3],
            ch5: n[4], ch6: n[5], ch7: n[6], ch8: n[7],

            // Legacy names – set 1 maps ch1..ch5 (shown on first display pass)
            delta:  n[0], theta:  n[1], alpha:  n[2], beta:  n[3], emg:  n[4],
            delta1: n[0], theta1: n[1], alpha1: n[2], beta1: n[3], emg1: n[4],

            // Legacy names – set 2 maps ch4..ch8 (shown on second display pass)
            delta2: n[3], theta2: n[4], alpha2: n[5], beta2: n[6], emg2: n[7],
        }

        //send data to device if device asks for it - we send it a couple times to be sure
        if (sendCommandFlag) {
            //this.standardServer = server;
            for (var i = 0; i < 3; i++) {
                //  sendControlService();
                _this.sendControlService([services.controlService, ], [characteristics.commandReadCharacteristic, characteristics.commandWriteCharacteristic, characteristics.deviceDataCharacteristic], _this.standardServer);
            }
            sendCommandFlag = false;
        }
        _this.onStateChangeCallback(state);
    }

    onStateChangeCallback() {}

    getdeviceData(service, characteristic, server) {
        return server.getPrimaryService(service.uuid)
            .then(newService => {
                console.log('getting characteristic: ', characteristic.name);
                return newService.getCharacteristic(characteristic.uuid)
            })
            .then(char => {
                char.startNotifications().then(res => {
                    char.addEventListener('characteristicvaluechanged', _this.handleDeviceDataChanged);
                })
            })
    }
    onStateChange(callback) {
        _this.onStateChangeCallback = callback;
    }
}

/*******************************************************************************************************************
 *********************************************** INITIALIZE *********************************************************
 ********************************************************************************************************************/
var testSignalFlag = false;

//sensor array sample data
var sensorDataArray = new Array(6).fill(0);

//master session data array of arrays
var sensorDataSession = [];

//master session data array of arrays
var sensorDataHistory = [];
//fill up two dimesional array
for(let i=0;i<10;i++){ sensorDataHistory.push(sensorDataArray); }

//sensor array sample data FOR CUSTOM TRAINING
var NNTrueDataArray = new Array;
var NNFalseDataArray = new Array;

var NNArchitecture = 'none';
var numInputs = 5;

var getSamplesFlag = 0;
var getSamplesTypeFlag = 0; //0=none 1=NN1T 2=NN1F 3=NN2T 4=NN2F

//do we have a trained NN to apply to live sensor data?
var haveNNFlag = false;
var trainNNFlag = false;
var activeNNFlag = false;

//NN scores
var scoreArray = new Array(1).fill(0);

var oldScore = 0;

var initialised = false;
var timeout = null;

function testSignal(){
	while(testSignalFlag){
		setInterval(function() {
			//dfdf
		}, 200); // throttle 200 = 5Hz limit
	}
}

$(document).ready(function() {

    /*******************************************************************************************************************
     *********************************************** WEB BLUETOOTH ******************************************************
     ********************************************************************************************************************/

    //Web Bluetooth connection button and ongoing device data update function
    button.onclick = function(e) {
        var sensorController = new ControllerWebBluetooth("hiibrarahmad-EEG");
        sensorController.connect();

        //on bluetooth notification value update ie new data over bluetooth
        sensorController.onStateChange(function(state) {
            bluetoothDataFlag = true;
        });

        //check for new data every X milliseconds - this is to decouple execution from Web Bluetooth actions
        setInterval(function() {
            //     bluetoothDataFlag = getBluetoothDataFlag();

            if (bluetoothDataFlag == true || secondDataSet == true) {

                timeStamp = new Date().getTime();

                //load data into global array
                sensorDataArray = new Array(6).fill(0);

                if(dataType == "FFT"){
        		//log base 10 values of eeg frequency bands are recieved so we have to take the inverse log to get our signal --> 10^x is inverse of log(x) base 10
			    	state.delta = Math.pow(10, state.delta);
			    	state.theta = Math.pow(10, state.theta);
			    	state.alpha = Math.pow(10, state.alpha);
			    	state.beta = Math.pow(10, state.beta);
			    	state.emg = Math.pow(10, state.emg);
			    } /*else if(dataType == "FIR"){ 
			    	state.delta = state.delta * 100;
			    	state.theta = state.theta * 100;
			    	state.alpha = state.alpha * 100;
			    	state.beta = state.beta * 100;
			    	state.emg = state.emg * 100;
			    } */
			    if(bluetoothDataFlag == true){
            		secondDataSet = true;
            	
	                sensorDataArray[0] = state.delta1.toFixed(3);
	                sensorDataArray[1] = state.theta1.toFixed(3);
	                sensorDataArray[2] = state.alpha1.toFixed(3);
	                sensorDataArray[3] = state.beta1.toFixed(3);
	                sensorDataArray[4] = state.emg1.toFixed(3);
	                sensorDataArray[5] = 0;       
	                sensorDataArray[6] = timeStamp;
	            } else if(secondDataSet == true){
	            	secondDataSet = false;

	            	sensorDataArray[0] = state.delta2.toFixed(3);
	                sensorDataArray[1] = state.theta2.toFixed(3);
	                sensorDataArray[2] = state.alpha2.toFixed(3);
	                sensorDataArray[3] = state.beta2.toFixed(3);
	                sensorDataArray[4] = state.emg2.toFixed(3);
	                sensorDataArray[5] = 0;       
	                sensorDataArray[6] = timeStamp;
	            }

            //update sensor data rolling history for variance and averages calculations
            for(let i = 0; i < 9; i++){
                sensorDataHistory[i] = sensorDataHistory[i + 1];
            }
            sensorDataHistory[9] = sensorDataArray;
         //   sensorDataHistory[9] = [runTimeData[runIndex][0].toFixed(3), runTimeData[runIndex][1].toFixed(3), runTimeData[runIndex][2].toFixed(3), runTimeData[runIndex][3].toFixed(3), runTimeData[runIndex][4].toFixed(3)];
          //  console.log("data history: " + sensorDataHistory);

            //calculate averages
            let sensorDataAverages = new Array(6).fill(0);
            for(let j = 0; j < 10; j++){
                for(let k = 0; k < 5; k++){
                    sensorDataAverages[k] = sensorDataAverages[k] + Number(sensorDataHistory[j][k]);
                   // console.log("data average total: " + sensorDataAverages[k] + "sensor data history item: " + sensorDataHistory[j][k]);
                }
            }
            for(let m = 0; m < 5; m++){
                sensorDataAverages[m] = sensorDataAverages[m] / 10;
            }

            //now that we have averages we can calculate variance from average
            let sensorDataVariance = new Array(6).fill(0);
            for(let j = 0; j < 10; j++){
                for(let k = 0; k < 5; k++){
                    sensorDataVariance[k] = sensorDataVariance[k] + Math.abs(sensorDataAverages[k] - Number(sensorDataHistory[j][k]) );
                }
            }
            for(let m = 0; m < 5; m++){
                sensorDataVariance[m] = (sensorDataVariance[m] / 10) / sensorDataAverages[m];
            }
          
            //node data selection
            let nodeCodes = new Array(5).fill(0);
            nodeCodes[0] = $('#code-node1').val();
            nodeCodes[1] = $('#code-node2').val();
            nodeCodes[2] = $('#code-node3').val();
            nodeCodes[3] = $('#code-node4').val();
            nodeCodes[4] = $('#code-node5').val();

          //  console.log("nodeCodes: " + nodeCodes[0] + " " + nodeCodes[1] + " " + nodeCodes[2] + " " + nodeCodes[3] + " " + nodeCodes[4]);

            let nodeDataTypes = new Array(5).fill(0);

            let nodeTempData = sensorDataArray; //for switching raw vals between nodes so we don't overwrite primary array
            for(let h = 0; h < 5; h++){
                //delta wave vals
                if(nodeCodes[h] == "DR"){      sensorDataArray[h] = nodeTempData[0];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "DA"){ sensorDataArray[h] = sensorDataAverages[0];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "DV"){ sensorDataArray[h] = sensorDataVariance[0];  nodeDataTypes[h] = "V"; }
                //theta wave vals
                else if(nodeCodes[h] == "TR"){ sensorDataArray[h] = nodeTempData[1];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "TA"){ sensorDataArray[h] = sensorDataAverages[1];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "TV"){ sensorDataArray[h] = sensorDataVariance[1];  nodeDataTypes[h] = "V"; }
                //alpha wave vals
                else if(nodeCodes[h] == "AR"){ sensorDataArray[h] = nodeTempData[2];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "AA"){ sensorDataArray[h] = sensorDataAverages[2];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "AV"){ sensorDataArray[h] = sensorDataVariance[2];  nodeDataTypes[h] = "V"; }
                //beta wave vals
                else if(nodeCodes[h] == "BR"){ sensorDataArray[h] = nodeTempData[3];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "BA"){ sensorDataArray[h] = sensorDataAverages[3];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "BV"){ sensorDataArray[h] = sensorDataVariance[3];  nodeDataTypes[h] = "V"; }
                //emg wave vals
                else if(nodeCodes[h] == "ER"){ sensorDataArray[h] = nodeTempData[4];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "EA"){ sensorDataArray[h] = sensorDataAverages[4];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "EV"){ sensorDataArray[h] = sensorDataVariance[4];  nodeDataTypes[h] = "V"; }
                else{ sensorDataArray[h] = 0; }
            } 


            console.log("Loaded Data: " + sensorDataArray[0] + " " + sensorDataArray[1] + " " + sensorDataArray[2] + " " + sensorDataArray[3] + " " + sensorDataArray[4] );

            //update time series chart with normalized values
            let rawChartData = new Array(5).fill(0);

            //modify graph appearence based on data type
             for(let q = 0; q < 5; q++){
                if(nodeDataTypes[q] == 'R'){
                    rawChartData[q] = (sensorDataArray[q] / 4) + (q) * 0.1;
                } else if( nodeDataTypes[q] == 'A'){
                    rawChartData[q] = (sensorDataArray[q] / 3) + (q) * 0.1;
                } else if (nodeDataTypes[q] == 'V'){
                    rawChartData[q] = (sensorDataArray[q] / 1) + (q) * 0.1;
                }
            }           

       /*     var rawDeltaChart = sensorDataArray[0];
            var rawThetaChart = sensorDataArray[1];
            var rawAlphaChart = sensorDataArray[2];
            var rawBetaChart  = sensorDataArray[3];
            var rawEMGChart  =  sensorDataArray[4];

            //sensor values in bottom 2/3 of chart , 1/10 height each
            rawDeltaChart = (rawDeltaChart / 4) + 4 * 0.1;
            rawThetaChart = (rawThetaChart / 4) + 3 * 0.1;
            rawAlphaChart = (rawAlphaChart / 4) + 2 * 0.1;
            rawBetaChart  = (rawBetaChart  / 4) + 1 * 0.1;
            rawEMGChart  = (rawEMGChart  / 4) + 0 * 0.1; */

            if(rawChartData[0] != 0) lineNode1.append(timeStamp, rawChartData[0]);
            if(rawChartData[1] != 0) lineNode2.append(timeStamp, rawChartData[1]);
            if(rawChartData[2] != 0) lineNode3.append(timeStamp, rawChartData[2]);
            if(rawChartData[3] != 0) lineNode4.append(timeStamp, rawChartData[3]);
            if(rawChartData[4] != 0) lineNode5.append(timeStamp, rawChartData[4]);  


           /*     //sensor values in bottom 2/3 of chart , 1/10 height each
                rawDeltaChart = (rawDeltaChart / 0.5) + 2 * 0.1;
                rawThetaChart = (rawThetaChart / 0.5) + 1 * 0.1;
                rawAlphaChart = (rawAlphaChart / 0.5) + 0 * 0.1;
                rawBetaChart  = (rawBetaChart  / 4) + 6 * 0.1;
                rawEMGChart  = (rawEMGChart  / 3) + 4 * 0.1; 

                lineNode1.append(timeStamp, rawDeltaChart);
                lineNode2.append(timeStamp, rawThetaChart);
                lineNode3.append(timeStamp, rawAlphaChart);
                lineNode4.append(timeStamp, rawBetaChart);
                lineNode5.append(timeStamp, rawEMGChart); */


                //if data sample collection has been flagged
                //  getSensorData();
                if (getSamplesFlag > 0) {
                    collectData();
                } else if (trainNNFlag) {
                    //don't do anything
                } else {
                    if (haveNNFlag && activeNNFlag) { //we have a NN and we want to apply to current sensor data
                        getNNScore();
                    } 
                }

                displayData();

                // ── Feed raw channel data to raw-view charts ─────────────────
                if (state.ch1 !== undefined) {
                    var rawTs = timeStamp;
                    // Convert normalised 0-1 back to raw 24-bit signed integer
                    // (norm = (raw + 8388608) / 16777215  →  raw = norm * 16777215 - 8388608)
                    for (var ri = 0; ri < 8; ri++) {
                        var rawVal = state['ch' + (ri + 1)] * 16777215 - 8388608;
                        rawAllChSeries[ri].append(rawTs, rawVal);
                        rawPerChSeries[ri].append(rawTs, rawVal);
                    }
                }

                bluetoothDataFlag = false;
            }

        }, 200); // throttle 200 = 5Hz limit
    }


    /*******************************************************************************************************************
    **************************************** STREAMING SENSOR DATA CHART ***********************************************
    *******************************************************************************************************************/

    //add smoothie.js time series streaming data chart
    var chartHeight = 350;
    var chartWidth = $(".streaming-data").width(); //$(window).width();

    $("#streaming-data-chart").html('<canvas id="chart-canvas" width="' + chartWidth + '" height="' + chartHeight + '"></canvas>');

    var streamingChart = new SmoothieChart({/*  grid: { strokeStyle:'rgb(125, 0, 0)', fillStyle:'rgb(60, 0, 0)', lineWidth: 1, millisPerLine: 250, verticalSections: 6, }, labels: { fillStyle:'rgb(60, 0, 0)' } */ });

    streamingChart.streamTo(document.getElementById("chart-canvas"), 300 /*delay*/ ); //delay by one second because data aquisition is slow

    var lineNode1 = new TimeSeries();
    var lineNode2 = new TimeSeries();
    var lineNode3 = new TimeSeries();
    var lineNode4 = new TimeSeries();
    var lineNode5 = new TimeSeries();
    var lineNN = new TimeSeries();


    streamingChart.addTimeSeries(lineNode1,  {strokeStyle: 'rgb(255, 255, 0)', lineWidth: 4 });
    streamingChart.addTimeSeries(lineNode2,  {strokeStyle: 'rgb(185, 76, 255)',   lineWidth: 4 });
    streamingChart.addTimeSeries(lineNode3,  {strokeStyle: 'rgb(255, 127, 0)',   lineWidth: 4 });
    streamingChart.addTimeSeries(lineNode4,   {strokeStyle: 'rgb(7, 185, 252)', lineWidth: 4 });
    streamingChart.addTimeSeries(lineNode5,    {strokeStyle: 'rgb(246, 70, 91)', lineWidth: 4 });
    streamingChart.addTimeSeries(lineNN,    {strokeStyle: 'rgb(72, 244, 68)',   lineWidth: 5 });


    //min/max streaming chart button
    $('#circleDrop').click(function() {

        $('.card-middle').slideToggle();
        $('.close').toggleClass('closeRotate');

        var chartHeight = $(window).height() / 1.2;
        var chartWidth = $(window).width();

        if ($("#chart-size-button").hasClass('closeRotate')) {
            $("#streaming-data-chart").html('<canvas id="chart-canvas" width="' + chartWidth + '" height="' + chartHeight + '"></canvas>');
        } else {
            $("#streaming-data-chart").html('<canvas id="chart-canvas" width="' + chartWidth + '" height="' + 100 + '"></canvas>');
        }

        //hide controls
        $("#basic-interface-container, #hand-head-ui-container, #nn-slide-controls, .console, #interface-controls, #dump-print, #record-controls").toggleClass("hide-for-chart");
        //redraw chart
        streamingChart.streamTo(document.getElementById("chart-canvas"), 350 /*delay*/ );
    });


    /*******************************************************************************************************************
    *************************** RAW CHANNEL CHARTS  (All-Channels + Per-Channel) **************************************
    *******************************************************************************************************************/

    var CH_COLORS = [
        'rgb(255,255,0)',   // CH1 yellow
        'rgb(185,76,255)',  // CH2 purple
        'rgb(255,127,0)',   // CH3 orange
        'rgb(7,185,252)',   // CH4 cyan
        'rgb(246,70,91)',   // CH5 red
        'rgb(72,244,68)',   // CH6 lime
        'rgb(255,0,255)',   // CH7 magenta
        'rgb(0,255,204)'    // CH8 teal
    ];

    // ── All-Channels chart (Tab 2) ──────────────────────────────────────────
    var allChW = Math.max(600, $(window).width() - 280);
    $("#allch-chart-container").html(
        '<canvas id="allch-canvas" width="' + allChW + '" height="380"></canvas>'
    );

    var allChChart = new SmoothieChart({
        minValue: -8500000, maxValue: 8500000,
        grid: { strokeStyle: 'rgba(255,255,255,0.08)', fillStyle: 'rgb(18,18,18)',
                lineWidth: 1, millisPerLine: 500, verticalSections: 8 },
        labels: { fillStyle: 'rgba(255,255,255,0.5)', fontSize: 11 },
        timestampFormatter: SmoothieChart.timeFormatter
    });
    allChChart.streamTo(document.getElementById("allch-canvas"), 300);

    var rawAllChSeries = [];
    for (var aci = 0; aci < 8; aci++) {
        var acSeries = new TimeSeries();
        allChChart.addTimeSeries(acSeries, { strokeStyle: CH_COLORS[aci], lineWidth: 2 });
        rawAllChSeries.push(acSeries);
    }

    // ── Per-Channel charts (Tab 3) ──────────────────────────────────────────
    var perChW = Math.max(500, $(window).width() - 320);
    var perChCharts = [];
    var rawPerChSeries = [];

    for (var pci = 0; pci < 8; pci++) {
        (function(idx) {
            var rowHtml =
                '<div class="perch-row">' +
                '<span class="perch-label" style="color:' + CH_COLORS[idx] + '">CH' + (idx + 1) + '</span>' +
                '<canvas id="perch-canvas-' + idx + '" width="' + perChW + '" height="130"></canvas>' +
                '</div>';
            $("#perch-charts-container").append(rowHtml);

            var pcChart = new SmoothieChart({
                minValue: -8500000, maxValue: 8500000,
                grid: { strokeStyle: 'rgba(255,255,255,0.06)', fillStyle: 'rgb(14,14,14)',
                        lineWidth: 1, millisPerLine: 500, verticalSections: 4 },
                labels: { fillStyle: 'rgba(255,255,255,0.4)', fontSize: 10 }
            });
            pcChart.streamTo(document.getElementById("perch-canvas-" + idx), 300);

            var pcSeries = new TimeSeries();
            pcChart.addTimeSeries(pcSeries, { strokeStyle: CH_COLORS[idx], lineWidth: 2 });

            perChCharts.push(pcChart);
            rawPerChSeries.push(pcSeries);
        })(pci);
    }


    //numerical data display
    function displayData() {
        var deltaElement =    document.getElementsByClassName('delta-data')[0];
        var thetaElement =    document.getElementsByClassName('theta-data')[0];
        var alphaElement =    document.getElementsByClassName('alpha-data')[0];
        var betaElement = 	  document.getElementsByClassName('beta-data')[0];
        var emgElement =     document.getElementsByClassName('emg-data')[0];

        if( $('#code-node1').val() == "DV" || $('#code-node1').val() == "TV" || $('#code-node1').val() == "AV" || $('#code-node1').val() == "BV" || $('#code-node1').val() == "EV"){
            deltaElement.innerHTML = (sensorDataArray[0] * 100).toFixed(2) + ' %';
        } else { deltaElement.innerHTML = ((sensorDataArray[0] * 160000) - 80000).toFixed(2); }

        if( $('#code-node2').val() == "DV" || $('#code-node2').val() == "TV" || $('#code-node2').val() == "AV" || $('#code-node2').val() == "BV" || $('#code-node2').val() == "EV"){
            thetaElement.innerHTML = (sensorDataArray[1] * 100).toFixed(2) + ' %';
        } else { thetaElement.innerHTML = ((sensorDataArray[1] * 160000) - 80000).toFixed(2); }

        if( $('#code-node3').val() == "DV" || $('#code-node3').val() == "TV" || $('#code-node3').val() == "AV" || $('#code-node3').val() == "BV" || $('#code-node3').val() == "EV"){
            alphaElement.innerHTML = (sensorDataArray[2] * 100).toFixed(2) + ' %';
        } else { alphaElement.innerHTML = ((sensorDataArray[2] * 160000) - 80000).toFixed(2); }

        if( $('#code-node4').val() == "DV" || $('#code-node4').val() == "TV" || $('#code-node4').val() == "AV" || $('#code-node4').val() == "BV" || $('#code-node4').val() == "EV"){
            betaElement.innerHTML = (sensorDataArray[3] * 100).toFixed(2) + ' %';
        } else { betaElement.innerHTML = ((sensorDataArray[3] * 160000) - 80000).toFixed(2); }

        if( $('#code-node5').val() == "DV" || $('#code-node5').val() == "TV" || $('#code-node5').val() == "AV" || $('#code-node5').val() == "BV" || $('#code-node5').val() == "EV"){
            emgElement.innerHTML = (sensorDataArray[4] * 100).toFixed(2) + ' %';
        } else { emgElement.innerHTML = ((sensorDataArray[4] * 160000) - 80000).toFixed(2); }

      /*  deltaElement.innerHTML =      	((sensorDataArray[0] * 160000) - 80000).toFixed(2);
        thetaElement.innerHTML =      	((sensorDataArray[1] * 160000) - 80000).toFixed(2);
        alphaElement.innerHTML =      	((sensorDataArray[2] * 160000) - 80000).toFixed(2);
        betaElement.innerHTML =  		((sensorDataArray[3] * 160000) - 80000).toFixed(2);
        emgElement.innerHTML =          ((sensorDataArray[4] * 160000) - 80000).toFixed(2); */
    }

    function collectData() {
        console.log("web bluetooth sensor data:");
        console.dir(sensorDataArray);

        //add sample to set
        sensorDataSession.push(sensorDataArray);

        if (getSamplesTypeFlag == 1) {
            NNTrueDataArray.push(sensorDataArray);
            $('.message-nn-true').html(NNTrueDataArray.length);
        } else if (getSamplesTypeFlag == 2) {
            NNFalseDataArray.push(sensorDataArray);
            $('.message-nn-false').html(NNFalseDataArray.length);
        } 

        //countdown for data collection
        getSamplesFlag = getSamplesFlag - 1;
    }


    /*******************************************************************************************************************
     *********************************************** NEURAL NETWORKS ****************************************************
     ********************************************************************************************************************/
    /**
     * Attach synaptic neural net components to app object
     */
    var nnRate =        $("#rate-input").val();
    var nnIterations =  $("#iterations-input").val();
    var nnError =       $("#error-input").val();

    // ************** NEURAL NET 
    var Neuron = synaptic.Neuron;
    var Layer = synaptic.Layer;
    var Network = synaptic.Network;
    var Trainer = synaptic.Trainer;
    var Architect = synaptic.Architect;

    //** LSTM Options etc. **/
      /*
var lstmOptions = {
        peepholes: Layer.connectionType.ALL_TO_ALL,
        hiddenToHidden: false,
        outputToHidden: false,
        outputToGates: false,
        inputToOutput: true,
    };
    var neuralNet = new Architect.LSTM(5, 5, 5, 1, lstmOptions);  
    */
    //LSTM options: https://github.com/cazala/synaptic/issues/217 
    //https://github.com/cazala/synaptic/issues/101
    //** END LSTM Options etc. **/

   var neuralNet = new Architect.Perceptron(5, 5, 1);
    var trainer = new Trainer(neuralNet);
    var trainingData;


    function getNNScore() {
        var feedArray = new Array(1).fill(0);
        var scoreArray = new Array(1).fill(0);
        var timeStamp = new Date().getTime();
        var displayScore;

        feedArray[0] = sensorDataArray[0];
        feedArray[1] = sensorDataArray[1];

        if (numInputs > 2) feedArray[2] = sensorDataArray[2];
        if (numInputs > 3) feedArray[3] = sensorDataArray[3];
        if (numInputs > 4) feedArray[4] = sensorDataArray[4];
            
        //make sure we are in bounds of normalization
        for(var k = 0; k < numInputs; k++){
        	if(feedArray[k] > 1) feedArray[k] = 1;
        	if(feedArray[k] < 0) feedArray[k] = 0;
        }

        // use trained NN or loaded NN
        if (haveNNFlag && activeNNFlag ) {
            scoreArray = neuralNet.activate(feedArray);
        }

        displayScore = (scoreArray[0].toFixed(4) * 100 + oldScore*2) / 3; //smooth
        oldScore = displayScore;
        displayScore = displayScore.toFixed(2);

        console.log("NN FEED ARRAY: " + feedArray);
        console.log("NN SCORE ARRAY: " + scoreArray);

        $(".message-nn-score").html(displayScore + '%');
        var rawlineNNChart = scoreArray[0].toFixed(4);
        rawlineNNChart = (rawlineNNChart / 3) + 0.6;
        lineNN.append(timeStamp, rawlineNNChart);
    }



    /**************************** TRAIN NN ******************************/
    function trainNN() {
        console.log("Training NN....");
        var processedDataSession = new Array;
        var falseDataArray = new Array;
        var trueDataArray = new Array;
        var combinedTrueFalse = new Array(13).fill(0);
        trainingData = new Array;

        var rawNNArchitecture = $("#nn-architecture").val();
        numInputs = parseInt(rawNNArchitecture.charAt(0)); 

        nnRate = $("#rate-input").val();
        nnIterations = $("#iterations-input").val();
        nnError = $("#error-input").val();

        trueDataArray = NNTrueDataArray;
        falseDataArray = NNFalseDataArray;


        //combine true and false data
        for (var j = 0; j < trueDataArray.length; j++) {
            combinedTrueFalse = trueDataArray[j];
            combinedTrueFalse[12] = 1; //true
            processedDataSession.push(combinedTrueFalse);
        }
        for (var k = 0; k < falseDataArray.length; k++) {
            combinedTrueFalse = falseDataArray[k];
            combinedTrueFalse[12] = 0; //false
            processedDataSession.push(combinedTrueFalse);
        }

        

        var getArchitect;
    /*    if (rawNNArchitecture == '2:1') {
            getArchitect = new Architect.LSTM(2, 1);
        } else if (rawNNArchitecture == '2:5:5:1') {
            getArchitect = new Architect.LSTM(2, 5, 5, 1);
        } else if (rawNNArchitecture == '3:1') {
            getArchitect = new Architect.LSTM(3, 1);
        } else if (rawNNArchitecture == '3:3:1') {
         //   getArchitect = new Architect.LSTM(3, 3, 1);
            getArchitect = new Architect.Perceptron(3, 3, 1);
        } else if (rawNNArchitecture == '3:3:3:1') {
        //    getArchitect = new Architect.LSTM(3, 3, 3, 1);
            getArchitect = new Architect.Perceptron(3, 3, 3, 1);
        } else if (rawNNArchitecture == '3:5:5:1') { */
           // getArchitect = new Architect.LSTM(4, 4, 4, 1);
            getArchitect = new Architect.Perceptron(5, 5, 1);
     //   } 


        neuralNet = getArchitect;
        NNArchitecture = rawNNArchitecture;
        trainer = new Trainer(neuralNet);


        //   console.log("raw NN architecture: " + rawNNArchitecture);

        //  console.log("SIZE OF UNPROCESSED SESSION DATA: " + processedDataSession.length);

        for (var i = 0; i < processedDataSession.length; i++) {

            var currentSample = processedDataSession[i];
            var outputArray = new Array(1).fill(0);
            var inputArray = new Array(2).fill(0);

            outputArray[0] = currentSample[12]; //true or false

            inputArray[0] = currentSample[0];
            inputArray[1] = currentSample[1];

            if (numInputs > 2) inputArray[2] = currentSample[2];
            if (numInputs > 3) inputArray[3] = currentSample[3];
            if (numInputs > 4) inputArray[4] = currentSample[4];

            //make sure we are in bounds of normalization
	        for(var k = 0; k < numInputs; k++){
	        	if(inputArray[k] > 1) inputArray[k] = 1;
	        	if(inputArray[k] < 0) inputArray[k] = 0;
	        }

            trainingData.push({
                input: inputArray,
                output: outputArray
            });

            console.log(currentSample + " TRAINING INPUT: " + inputArray);
            console.log(currentSample + " TRAINING OUTPUT: " + outputArray);
        }



            console.log("TRAINING interations:" + nnIterations + "  error:" + nnError + "  rate:" + nnRate + "  arch:" + rawNNArchitecture + "  inputs:" + numInputs);

            trainer.train(trainingData, {
                rate: nnRate,
                //   iterations: 15000,
                iterations: nnIterations,
                error: nnError,
                shuffle: true,
                //   log: 1000,
                log: 5,
                cost: Trainer.cost.CROSS_ENTROPY
            });

            //we have a trained NN to use
            haveNNFlag = true;
            trainNNFlag = false;
            $('#activate-btn').addClass("haveNN");
            $('#export-btn').addClass("haveNN");
    }


    /*******************************************************************************************************************
     ******************************************* NEURAL NETWORK BUTTONS *************************************************
     ********************************************************************************************************************/
    $('#train-btn').click(function() {
        console.log("train button");
        trainNNFlag = true;
        trainNN();
    });

    $('#activate-btn').click(function() {
        console.log("activate button");
        activeNNFlag = true;
        $('#activate-btn').toggleClass("activatedNN");
    });


    /*******************************************************************************************************************
     ********************************** COLLECT, PRINT, LOAD BUTTON ACTIONS *********************************************
     ********************************************************************************************************************/

    /*************** COLLECT SAMPLE - SONSOR AND MODEL DATA - STORE IN GSHEET AND ADD TO NN TRAINING OBJECT *****************/
    $('#collect-true').click(function() {
        //how many samples for this set?
        getSamplesFlag = $('input.sample-size').val();
        getSamplesTypeFlag = 1;
        console.log("Collect btn NN T #samples flag: " + getSamplesFlag);
    });

    $('#collect-false').click(function() {
        //how many samples for this set?
        //this flag is applied in the bluetooth data notification function
        getSamplesFlag = $('input.sample-size').val();
        getSamplesTypeFlag = 2;
        console.log("Collect btn NN F #samples flag: " + getSamplesFlag);
    });


    $('#clear').click(function() {
        clearData();
    });

    function clearData(){
        NNTrueDataArray = new Array;
        NNFalseDataArray = new Array;
        sensorDataArray = new Array(18).fill(0);
        sensorDataSession = new Array;
        $('.message-nn-true').html('');
        $('.message-nn-false').html('');
        $("#dump-print").html("");
        console.log("Clear NN Data");
    }


    $('#export-btn').click(function() {
        console.log("export NN button");
        //clear everything but key values from stored NN
        neuralNet.clear();

        //export optimized weights and activation function
        var standalone = neuralNet.standalone();

        //convert to string for parsing
        standalone = standalone.toString();

        console.log(standalone);
        $("#dump-print").html(standalone);
        $("#dump-print").addClass("active-print");
    });


    //connect button handler
    $('#connect').click(function() {
        console.log("connect button");

        //"active-data" class controls active data type
        $('#connect').addClass('active-data');
        //turn off any recorded test data
        $('#false-test-data').removeClass('active-data');
        $('#true-test-data').removeClass('active-data');

    });

    //true test data button handler
    $('#true-test-data').click(function() {
        console.log("true test data button");

        //"active-data" class controls active data type
        $('#true-test-data').addClass('active-data');
        $('#connect').removeClass('active-data');
        $('#false-test-data').removeClass('active-data');

        //load and run test data
     //   runData();
    });

    //false test data button handler
    $('#false-test-data').click(function() {
        console.log("false test data button");

        //"active-data" class controls active data type
        $('#false-test-data').addClass('active-data');
        $('#connect').removeClass('active-data');
        $('#true-test-data').removeClass('active-data');

        //load and run test data
      //  runData();
    });

    function runData(){

        let runTimeData;
        let currentDataType;
        let runIndex = 0;
        let testDataLength = 0;

        var runDataHandle = setInterval(function() {

            //are we using test data, and if so what data
            if ( $('#true-test-data').hasClass('active-data') ){
                runTimeData = eegTrueTestData;
                testDataLength = eegTrueTestData.length;
                currentDataType = 'test-true';
            }else if( $('#false-test-data').hasClass('active-data') ){
                runTimeData = eegFalseTestData;
                testDataLength = eegFalseTestData.length;
                currentDataType = 'test-false';
            } else{
                console.log("terminating test data.....");
                clearInterval(runDataHandle);
                return;
            }

            timeStamp = new Date().getTime();

            if(runIndex >= testDataLength){ runIndex = 0; } else { runIndex++; }

            //load data into global array
            sensorDataArray = new Array(6).fill(0);

            //load data array
            sensorDataArray[0] = runTimeData[runIndex][0].toFixed(3);
            sensorDataArray[1] = runTimeData[runIndex][1].toFixed(3);
            sensorDataArray[2] = runTimeData[runIndex][2].toFixed(3);
            sensorDataArray[3] = runTimeData[runIndex][3].toFixed(3);
            sensorDataArray[4] = runTimeData[runIndex][4].toFixed(3);
            sensorDataArray[5] = 0;       
            sensorDataArray[6] = timeStamp;

            //update sensor data rolling history for variance and averages calculations
            for(let i = 0; i < 9; i++){
                sensorDataHistory[i] = sensorDataHistory[i + 1];
            }
          //  sensorDataHistory[9] = sensorDataArray;
            sensorDataHistory[9] = [runTimeData[runIndex][0].toFixed(3), runTimeData[runIndex][1].toFixed(3), runTimeData[runIndex][2].toFixed(3), runTimeData[runIndex][3].toFixed(3), runTimeData[runIndex][4].toFixed(3)];
          //  console.log("data history: " + sensorDataHistory);

            //calculate averages
            let sensorDataAverages = new Array(6).fill(0);
            for(let j = 0; j < 10; j++){
                for(let k = 0; k < 5; k++){

                    //use raw test data if possible
                    if(runIndex > 10){
                        sensorDataAverages[k] = sensorDataAverages[k] + Number( runTimeData[runIndex - j][k] ); // Number(sensorDataHistory[j][k]);
                    } else {
                        sensorDataAverages[k] = sensorDataAverages[k] + Number(sensorDataHistory[j][k]);
                    }
                   // console.log("data average total: " + sensorDataAverages[k] + "sensor data history item: " + sensorDataHistory[j][k]);
                }
            }
            for(let m = 0; m < 5; m++){
                sensorDataAverages[m] = sensorDataAverages[m] / 10;
            }

            //now that we have averages we can calculate variance from average
            let sensorDataVariance = new Array(6).fill(0);
            for(let j = 0; j < 10; j++){
                for(let k = 0; k < 5; k++){

                    //use raw test data if possible
                    if(runIndex > 10){
                        sensorDataVariance[k] = sensorDataVariance[k] + Math.abs(sensorDataAverages[k] - Number(runTimeData[runIndex - j][k]) );
                    } else {
                        sensorDataVariance[k] = sensorDataVariance[k] + Math.abs(sensorDataAverages[k] - Number(sensorDataHistory[j][k]) );
                    }
                }
            }
            for(let m = 0; m < 5; m++){
                sensorDataVariance[m] = (sensorDataVariance[m] / 10) / sensorDataAverages[m];
            }
          
            //node data selection
            let nodeCodes = new Array(5).fill(0);
            nodeCodes[0] = $('#code-node1').val();
            nodeCodes[1] = $('#code-node2').val();
            nodeCodes[2] = $('#code-node3').val();
            nodeCodes[3] = $('#code-node4').val();
            nodeCodes[4] = $('#code-node5').val();

          //  console.log("nodeCodes: " + nodeCodes[0] + " " + nodeCodes[1] + " " + nodeCodes[2] + " " + nodeCodes[3] + " " + nodeCodes[4]);

            let nodeDataTypes = new Array(5).fill(0);

            let nodeTempData = sensorDataArray; //for switching raw vals between nodes so we don't overwrite primary array
            for(let h = 0; h < 5; h++){
                //delta wave vals
                if(nodeCodes[h] == "DR"){      sensorDataArray[h] = nodeTempData[0];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "DA"){ sensorDataArray[h] = sensorDataAverages[0];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "DV"){ sensorDataArray[h] = sensorDataVariance[0];  nodeDataTypes[h] = "V"; }
                //theta wave vals
                else if(nodeCodes[h] == "TR"){ sensorDataArray[h] = nodeTempData[1];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "TA"){ sensorDataArray[h] = sensorDataAverages[1];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "TV"){ sensorDataArray[h] = sensorDataVariance[1];  nodeDataTypes[h] = "V"; }
                //alpha wave vals
                else if(nodeCodes[h] == "AR"){ sensorDataArray[h] = nodeTempData[2];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "AA"){ sensorDataArray[h] = sensorDataAverages[2];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "AV"){ sensorDataArray[h] = sensorDataVariance[2];  nodeDataTypes[h] = "V"; }
                //beta wave vals
                else if(nodeCodes[h] == "BR"){ sensorDataArray[h] = nodeTempData[3];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "BA"){ sensorDataArray[h] = sensorDataAverages[3];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "BV"){ sensorDataArray[h] = sensorDataVariance[3];  nodeDataTypes[h] = "V"; }
                //emg wave vals
                else if(nodeCodes[h] == "ER"){ sensorDataArray[h] = nodeTempData[4];        nodeDataTypes[h] = "R"; }
                else if(nodeCodes[h] == "EA"){ sensorDataArray[h] = sensorDataAverages[4];  nodeDataTypes[h] = "A"; }
                else if(nodeCodes[h] == "EV"){ sensorDataArray[h] = sensorDataVariance[4];  nodeDataTypes[h] = "V"; }
                else{ sensorDataArray[h] = 0; }
            } 


            console.log("Loaded Data: " + sensorDataArray[0] + " " + sensorDataArray[1] + " " + sensorDataArray[2] + " " + sensorDataArray[3] + " " + sensorDataArray[4] );

            //update time series chart with normalized values
            let rawChartData = new Array(5).fill(0);

            //modify graph appearence based on data type
             for(let q = 0; q < 5; q++){
                if(nodeDataTypes[q] == 'R'){
                    rawChartData[q] = (sensorDataArray[q] / 4) + (q) * 0.1;
                } else if( nodeDataTypes[q] == 'A'){
                    rawChartData[q] = (sensorDataArray[q] / 3) + (q) * 0.1;
                } else if (nodeDataTypes[q] == 'V'){
                    rawChartData[q] = (sensorDataArray[q] / 1) + (q) * 0.1;
                }
            }           

       /*     var rawDeltaChart = sensorDataArray[0];
            var rawThetaChart = sensorDataArray[1];
            var rawAlphaChart = sensorDataArray[2];
            var rawBetaChart  = sensorDataArray[3];
            var rawEMGChart  =  sensorDataArray[4];

            //sensor values in bottom 2/3 of chart , 1/10 height each
            rawDeltaChart = (rawDeltaChart / 4) + 4 * 0.1;
            rawThetaChart = (rawThetaChart / 4) + 3 * 0.1;
            rawAlphaChart = (rawAlphaChart / 4) + 2 * 0.1;
            rawBetaChart  = (rawBetaChart  / 4) + 1 * 0.1;
            rawEMGChart  = (rawEMGChart  / 4) + 0 * 0.1; */

            if(rawChartData[0] != 0) lineNode1.append(timeStamp, rawChartData[0]);
            if(rawChartData[1] != 0) lineNode2.append(timeStamp, rawChartData[1]);
            if(rawChartData[2] != 0) lineNode3.append(timeStamp, rawChartData[2]);
            if(rawChartData[3] != 0) lineNode4.append(timeStamp, rawChartData[3]);
            if(rawChartData[4] != 0) lineNode5.append(timeStamp, rawChartData[4]);

            //if data sample collection has been flagged
            if (getSamplesFlag > 0) {
                collectData();
            } else if (trainNNFlag) {
                //don't do anything
            } else {
                if (haveNNFlag && activeNNFlag) { //we have a NN and we want to apply to current sensor data
                    getNNScore();
                } 
            }

            displayData();
            bluetoothDataFlag = false;

        }, 200); // throttle 200 = 5Hz limit
    }

     /*******************************************************************************************************************
     *********************************************** SLIDER UI ******************************************************
     ********************************************************************************************************************/
    var rangeSlider = function(){
        var slider = $('.range-slider'),
            range = $('.range-slider__range'),
            value = $('.range-slider__value');
          
        slider.each(function(){

        value.each(function(){
            var value = $(this).prev().attr('value');
            $(this).html(value);
        });

        if( $(this).hasClass('nn-architecture') ){ $('.range-slider__value.nn-architecture').html('4:4:4:1'); }

        range.on('input', function(){
            var labels = ['2:1', '3:4:4:1', '4:1', '4:4:1', '4:3:3:1', '4:4:4:1'];
            $(this).next(value).html(this.value);

            if( $(this).hasClass('nn-architecture') ){ $(this).next(value).html( labels[this.value] ); }
          
          }); 
        }); 
    }

    rangeSlider();

    //RANGE SLIDER EVENT HANDLER
    $( ".range-slider" ).each(function() {

      //  if($(this).hasClass("nn-architecture")){
            // Add labels to slider whose values 
            // are specified by min, max and whose
            // step is set to 1
            
            // Get the options for this slider
            //var opt = $(this).data().uiSlider.options;
            // Get the number of possible values
            var $input = $(this).find("input");
            var min = parseInt($input.attr("min"));
            var max = parseInt($input.attr("max"));
            var step = parseInt($input.attr("step"));
            var increment = parseInt($input.attr("increment"));
            var vals = max - min; //opt.max - opt.min;
            //if(min < 0){ vals = max + min; }
          //  var labels = ['2:1', '3:4:4:1', '4:1', '4:3:1', '4:3:3:1', '4:4:4:1'];
            
            // Space out values
            for (var i = 0; (i * increment) <= vals; i++) {
                var s = min + (i * increment);
                var el = $('<label>'+ labels[s] +'</label>').css('left',( 4 + Math.abs((s-min)/vals) *($input.width() -24)+'px'));
                //   var el = $('<label>'+ s +'</label>').css('left',( 3 + ((s-min)/vals) *($input.width() -24)+'px'));
                if(s == 0){ el = $('<label>'+ labels[s] +'</label>').css('left',( 21 + Math.abs((s-min)/vals) *($input.width() -24)+'px')); }
                if(s == vals){ el = $('<label>'+ labels[s] +'</label>').css('left',( -20 + Math.abs((s-min)/vals) *($input.width() -24)+'px')); }
                $(this).append(el);
            }
      //  }  
    });

    //TEST DATA AUTOMATICALLY LOADS WHEN SITE LOADS
    $(document).ready(function() {
        $('#true-test-data').addClass('active-data');
        $('#connect','#false-test-data').removeClass('active-data');

        //load and run test data
        runData();
    });

}); // end on document load
