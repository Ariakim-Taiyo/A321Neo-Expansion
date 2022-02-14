let tiltToHold = 0;
let ils = true;
let deadZone = 0.01;
let pitchCenter = 0;
let rollTohold = 0;
let lastlla = [];
let velocityVec = 0;
// Converts from degrees to radians.
function toRadians(degrees) {
  return degrees * Math.PI / 180;
};

// Converts from radians to degrees.
function toDegrees(radians) {
  return radians * 180 / Math.PI;
}


function bearing(startLat, startLng, destLat, destLng) {
  startLat = toRadians(startLat);
  startLng = toRadians(startLng);
  destLat = toRadians(destLat);
  destLng = toRadians(destLng);

  y = Math.sin(destLng - startLng) * Math.cos(destLat);
  x = Math.cos(startLat) * Math.sin(destLat) -
    Math.sin(startLat) * Math.cos(destLat) * Math.cos(destLng - startLng);
  brng = Math.atan2(y, x);
  brng = toDegrees(brng);
  return (brng + 360) % 360;
}
function getVV() {
  velocityVec = bearing(lastlla[0], lastlla[1], geofs.aircraft.instance.llaLocation[0], geofs.aircraft.instance.llaLocation[1])
}

function computePitch() {
  // implement tilt holding
  if (geofs.animation.values.pitch <= deadZone && geofs.animation.values.pitch >= -deadZone) {
    pitchStage1 = -(tiltToHold - geofs.animation.values.atilt) / 10
    pitchCenter = pitchStage1;
  }
  else {
    //stall protection
    if (geofs.animation.values.aoa <= 10) {
      pitchStage1 = -(tiltToHold - geofs.animation.values.atilt) / 10
      tiltToHold = geofs.animation.values.atilt - geofs.animation.values.pitch * 5
    }
    else {
      pitchStage1 = -(tiltToHold - geofs.animation.values.atilt) / 10
      tiltToHold = (geofs.animation.values.atilt - geofs.animation.values.pitch * 5) + 10
    }
  }
  geofs.animation.values.computedPitch = clamp(pitchStage1 + pitchCenter, -1, 1)


}

function computeRoll() {
  //roll stabilization from input
  if (rollTohold <= 30 && rollTohold >= -30) {
    rollTohold = rollTohold - geofs.animation.values.roll / 5;
    geofs.animation.values.computedRoll = clamp((geofs.animation.values.aroll - rollTohold) / 10, -1, 1);
  }
  else {
    if (geofs.animation.values.aroll >= 0) {
      rollTohold = 29;
      geofs.animation.values.computedRoll = clamp((geofs.animation.values.aroll - rollTohold) / 10, -1, 1)
    }
    if (geofs.animation.values.aroll <= 0) {
      rollTohold = -29;
      geofs.animation.values.computedRoll = clamp((geofs.animation.values.aroll - rollTohold) / 10, -1, 1)
    }
  }
}
function computeYaw() {
  //yaw damper experiment. probably not good for crosswinds lol
  geofs.animation.values.computedYaw = geofs.animation.values.yaw
}

// get running average to dampen control inputs
let pitchInputs = [0, 0, 0, 0, 0, 0, 0];
let rollInputs = [0, 0, 0, 0, 0, 0, 0];
let yawInputs = [0, 0, 0, 0, 0, 0, 0];
geofs.animation.values.averagePitch = null;
geofs.animation.values.averageRoll = null;
geofs.animation.values.averageYaw = null;
geofs.animation.values.outerAveragePitch = null;
geofs.animation.values.outerAverageRoll = null;
geofs.animation.values.outerAverageYaw = null;
function pushInputs() {
  pitchInputs.push(geofs.animation.values.computedPitch);
  rollInputs.push(geofs.animation.values.computedRoll);
  yawInputs.push(geofs.animation.values.computedYaw);
}
function computeOutputs() {
  var pitchcheck = movingAvg(pitchInputs, 2, 2);
  var rollcheck = movingAvg(rollInputs, 2, 2);
  var yawcheck = movingAvg(yawInputs, 4, 4);
  geofs.animation.values.averagePitch = pitchcheck[pitchcheck.length - 3]
  geofs.animation.values.averageRoll = rollcheck[rollcheck.length - 3];
  geofs.animation.values.averageYaw = yawcheck[yawcheck.length - 3];
  geofs.animation.values.outerAveragePitch = clamp(geofs.animation.values.averagePitch / (geofs.animation.values.kias / 200), -1, 1);
  geofs.animation.values.outerAverageRoll = clamp(geofs.animation.values.averageRoll / (geofs.animation.values.kias / 100), -1, 1);
  geofs.animation.values.outerAverageYaw = clamp(geofs.animation.values.averageYaw / (geofs.animation.values.kias / 250), -1, 1);
}

function movingAvg(array, countBefore, countAfter) {
  if (countAfter == undefined) countAfter = 0;
  const result = [];
  for (let i = 0; i < array.length; i++) {
    const subArr = array.slice(Math.max(i - countBefore, 0), Math.min(i + countAfter + 1, array.length));
    const avg = subArr.reduce((a, b) => a + (isNaN(b) ? 0 : b), 0) / subArr.length;
    result.push(avg);
  }
  return result;
}
geofs.aircraft.instance.parts.elevleft.animations[0].value = "outerAveragePitch"
geofs.aircraft.instance.parts.elevright.animations[0].value = "outerAveragePitch"
geofs.aircraft.instance.parts.aileronleft.animations[0].value = "averageRoll"
geofs.aircraft.instance.parts.aileronright.animations[0].value = "averageRoll"
geofs.aircraft.instance.parts.rudder.animations[0].value = "averageYaw";

setInterval(function() {
  getVV()
  computeRoll();
  computePitch();
  computeYaw();
  pushInputs();
  computeOutputs();
}, 20)
setInterval(function() {
  lastlla = geofs.aircraft.instance.llaLocation
}, 200)

//engine simulations
let apu = false;
let engineL = false;
let engineR = false;
let ptuBork = false;
let hydraulicsPL = 0;
let hydraulicsPR = 0;
let engRon = false;
let engLon = false;
let electricalOn = false;
let landingLightsOn = false;
let strobeOn = false;
let terrainGPWS = false;
geofs.animation.values.rpmL = 0;
geofs.animation.values.rpmR = 0;
geofs.animation.values.rpmLfan = 0;
geofs.animation.values.rpmRfan = 0;
geofs.animation.values.flapschange = 0;
geofs.animation.values.ptu = 0;

geofs.aircraft.instance.parts.fanleft.animations[0].value = "rpmLfan"
geofs.aircraft.instance.parts.fanright.animations[0].value = "rpmRfan"

function engineTest() {
  if (engineL) {
    geofs.aircraft.instance.engines[0].thrust = 147240;
  }
  else {
    geofs.aircraft.instance.engines[0].thrust = 0;
  }

  if (engineR) {
    geofs.aircraft.instance.engines[1].thrust = 147240;
  }
  else {
    geofs.aircraft.instance.engines[1].thrust = 0;
  }
};

function getHyraulics() {
  if (engineL || engLon) {
    hydraulicsPL = clamp(geofs.animation.values.rpmL, 0, 1000) / 2;
  }
  else {
    hydraulicsPL = 0;
  }
  if (engineR || engRon) {
    hydraulicsPR = clamp(geofs.animation.values.rpmR, 0, 1000) / 2;
  }
  else {
    hydraulicsPR = 0;
  }
}
function getRPMs() {
  if (engLon) {
    geofs.animation.values.rpmL = geofs.animation.values.rpmL + 2.5;
    if (geofs.animation.values.rpmL >= 1000) {
      engLon = false;
      engineL = true;
    }
  }
  if (engineL) {
    geofs.animation.values.rpmL = geofs.animation.values.rpm + 100;
  }
  if (engRon) {
    geofs.animation.values.rpmR = geofs.animation.values.rpmR + 2.5;
    if (geofs.animation.values.rpmR >= 1000) {
      engRon = false;
      engineR = true;
    }
  }
  if (engineR) {
    geofs.animation.values.rpmR = geofs.animation.values.rpm + 100;
  }
  geofs.animation.values.rpmLfan = geofs.animation.values.rpmL * geofs.api.viewer.clock.currentTime.secondsOfDay / 100;
  geofs.animation.values.rpmRfan = geofs.animation.values.rpmR * geofs.api.viewer.clock.currentTime.secondsOfDay / 100;
}

function getPTUBork() {
  if (engLon || engRon) {
    if (Math.abs(hydraulicsPL - hydraulicsPR) >= 100) {
      ptuBork = true;
      geofs.animation.values.ptu = 1;
    }
    else {
      ptuBork = false;
      geofs.animation.values.ptu = 0;
    }
  }
  else {
    ptuBork = false;
    geofs.animation.values.ptu = 0;
  }
}

function startEngine(a) {
  audio.playStartup();
  if (a == "left" && geofs.animation.values.rpmL <= 999) {
    engLon = true;
  }
  if (a == "right" && geofs.animation.values.rpmR <= 999) {
    engRon = true;
  }
}

function stopEngines() {
  engineL = false;
  engineR = false;
  geofs.animation.values.rpmL = 0;
  geofs.animation.values.rpmR = 0;


}
simInterval = setInterval(function() {
  getHyraulics();
  getPTUBork()
  getRPMs();
  getFlapChange();
  engineTest();
}, 15)

//sound additions

geofs.aircraft.instance.definition.sounds[0].effects.volume.value = "rpmL";
geofs.aircraft.instance.definition.sounds[0].effects.pitch.value = "rpmL";
geofs.aircraft.instance.definition.sounds[0].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/neo2.mp3"
geofs.aircraft.instance.definition.sounds[1].effects.volume.value = "rpmL";
geofs.aircraft.instance.definition.sounds[1].effects.pitch.value = "rpmL";
geofs.aircraft.instance.definition.sounds[1].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/neo2.mp3"
geofs.aircraft.instance.definition.sounds[2].effects.volume.value = "rpmL";
geofs.aircraft.instance.definition.sounds[2].effects.pitch.value = "rpmL";
geofs.aircraft.instance.definition.sounds[2].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/neo3.mp3"
geofs.aircraft.instance.definition.sounds[5].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/startupneo.mp3"

geofs.aircraft.instance.definition.sounds[7] = {}
geofs.aircraft.instance.definition.sounds[7].id = "rpm4"
geofs.aircraft.instance.definition.sounds[7].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/neo2.mp3"
geofs.aircraft.instance.definition.sounds[7].effects = { "volume": { "value": "rpmR", "ramp": [800, 950, 2500, 3500], "ratio": 1 }, "pitch": { "value": "rpmR", "ramp": [0, 20000, 20000, 20000], "offset": 1, "ratio": 1 } };
geofs.aircraft.instance.definition.sounds[8] = {}
geofs.aircraft.instance.definition.sounds[8].id = "rpm5"
geofs.aircraft.instance.definition.sounds[8].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/neo2.mp3"
geofs.aircraft.instance.definition.sounds[8].effects = { "volume": { "value": "rpmR", "ramp": [1000, 2500, 10000, 10000], "ratio": 1 }, "pitch": { "value": "rpmR", "ramp": [0, 20000, 20000, 20000], "offset": 1, "ratio": 1.5 } };
geofs.aircraft.instance.definition.sounds[9] = {}
geofs.aircraft.instance.definition.sounds[9].id = "rpm6"
geofs.aircraft.instance.definition.sounds[9].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/neo3.mp3"
geofs.aircraft.instance.definition.sounds[9].effects = { "volume": { "value": "rpmR", "ramp": [6000, 20000, 20000, 20000], "ratio": 1 }, "pitch": { "value": "rpmR", "ramp": [1000, 20000, 20000, 20000], "offset": 1, "ratio": 1.5 } };

geofs.aircraft.instance.definition.sounds[10] = {}
geofs.aircraft.instance.definition.sounds[10].id = "ptuBork"
geofs.aircraft.instance.definition.sounds[10].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/ptu.mp3"
geofs.aircraft.instance.definition.sounds[10].effects = { "start": { "value": "ptu" } };



//I will implement the UI later once its done. For now just call startEngine(); for both engines.

//GPWS and RadioAlt callout test

let isApprConfig = false;
let trafficAlt = null;
geofs.animation.values.isFlapsWarn = 0;
geofs.animation.values.isGearWarn = 0;
geofs.animation.values.isTerrainWarn = 0;
geofs.animation.values.isPullupWarn = 0;
geofs.animation.values.isBankWarn = 0;
geofs.animation.values.isTCASClimb = 0;
geofs.animation.values.isTCASDescend = 0;
geofs.animation.values.isTCAS = 0;
geofs.animation.values.isTCASClear = 0;
geofs.animation.values.is100ab = 0;
geofs.animation.values.isWindWarn = 0;
geofs.animation.values.gpws1000 = 0;
geofs.animation.values.gpws500 = 0;
geofs.animation.values.gpws400 = 0;
geofs.animation.values.gpws300 = 0;
geofs.animation.values.gpws200 = 0;
geofs.animation.values.gpws100 = 0;
geofs.animation.values.gpws50 = 0;
geofs.animation.values.gpws40 = 0;
geofs.animation.values.gpws30 = 0;
geofs.animation.values.gpws20 = 0;
geofs.animation.values.gpws10 = 0;
geofs.animation.values.isRetard = 0;
let restingPoint = 12.232906828403847;

function getTrafficProximity() {
  Object.values(multiplayer.visibleUsers).forEach(function(e) {
    if (e.distance <= 1000) {
      if (e.referencePoint.lla[2] >= geofs.animation.values.altitudeMeters && e.referencePoint.lla[2] <= geofs.animation.values.altitudeMeters + 100) {
        geofs.animation.values.isTCASDescend = 1;
        if (geofs.animation.values.isTCAS == 0) {
          let tcasInt = setInterval(function(i) {
            if (i <= 2) {
              geofs.animation.values.isTCAS = 1;
            }
            else {
              geofs.animation.values.isTCAS = null;
            }
          }, 1000)
        }
        else {
          geofs.animation.values.isTCASDescend = 0;
        }
        if (e.referencePoint.lla[2] <= geofs.animation.values.altitudeMeters && e.referencePoint.lla[2] >= geofs.animation.values.altitudeMeters - 100) {
          geofs.animation.values.isTCASClimb = 1;
          if (geofs.animation.values.isTCAS == 0) {
            let tcasInt = setInterval(function(i) {
              if (i <= 2) {
                geofs.animation.values.isTCAS = 1;
              }
              else {
                geofs.animation.values.isTCAS = null;
              }
            }, 1000)
          }
        }
        else {
          geofs.animation.values.isTCASClimb = 0;
        }
      }
      else {
        if (geofs.animation.values.isTCASClimb == 1 || geofs.animation.values.isTCASDescend == 1) {
          counterAtClear = counter
          geofs.animation.values.isTCASClimb = 0;
          geofs.animation.values.isTCASDescend = 0;
          geofs.animation.values.isTCAS = 0;

          let counterInterval = setInterval(function(i) {
            if (i <= 3) {
              geofs.animation.values.isTCASClear = 1;
            }
            else {
              geofs.animation.values.isTCASClear = 0;
              clearinterval(counterInterval);
            }
          }, 1000)
        }
      }
    }
  })
}

let counter = 0;
let counterAtClear = 0;
let lastWind = 0;
let interval = setInterval(function() {
  if (counter < 2) {
    counter = counter + 1
  }
  else { counter = 0 }
  if (counter == 1) {
    lastWind = weather.currentWindSpeed;
  }

}, 1000)
function getWindShear() {
  if (isApprConfig == 1 && lastWind - weather.currentWindSpeed <= 5) {
    geofs.animation.values.isWindWarn = 1;
  }
  else {
    geofs.animation.values.isWindWarn = 0;
  }
}

function getGearFlapsWarn() {

  if (geofs.animation.values.haglFeet <= 500 && geofs.animation.values.gearPosition == 1 && geofs.animation.values.climbrate < 0 && geofs.animation.values.isPullupWarn == 0) {
    geofs.animation.values.isGearWarn = 1
  }
  else {
    geofs.animation.values.isGearWarn = 0
  }

  if (geofs.animation.values.haglFeet <= 1000 && geofs.animation.values.flapsPosition == 0 && geofs.animation.values.climbrate < 0 && geofs.animation.values.isPullupWarn == 0) {
    geofs.animation.values.isFlapsWarn = 1
  }
  else {
    geofs.animation.values.isFlapsWarn = 0
  }
}

function testTerrainorAppr() {
if (geofs.animation.values.gearPosition == 0) {
  if (geofs.animation.values.haglFeet <= 1000 && geofs.animation.values.climbrate <= -100 && geofs.animation.values.climbrate >= -5000 && geofs.animation.values.isGearWarn == 0 && geofs.animation.values.isFlapsWarn == 0 && isApprConfig == 0) {
    geofs.animation.values.isTerrainWarn = 1;
  }
  else {
    geofs.animation.values.isTerrainWarn = 0;
  }

  if (geofs.animation.values.haglFeet <= 5000 && geofs.animation.values.climbrate <= -2000 || geofs.animation.values.haglFeet <= 1000 && geofs.animation.values.climbrate <= -5000) {
    geofs.animation.values.isPullupWarn = 1;
  }
  else {
    geofs.animation.values.isPullupWarn = 0;
  }
}
  else {
    geofs.animation.values.isTerrainWarn = 0;
    return
  }
}

function getFlapChange() {
  if (geofs.animation.values.flapsPosition < geofs.animation.values.flapsTarget || geofs.animation.values.flapsPosition > geofs.animation.values.flapsTarget) {
    console.log("flaps extend")
    geofs.animation.values.flapschange = 1
  }
  else {
    geofs.animation.values.flapschange = 0
  }
}

function testForApproach() {
  if (geofs.animation.values.isFlapsWarn == 0 && geofs.animation.values.isGearWarn == 0 && geofs.animation.values.climbrate <= -1) {
    isApprConfig = true
  }
  else {
    isApprConfig = false
  }
}

function getRetard() {
  if (geofs.animation.values.gpws20 == 1 || geofs.animation.values.gpws10 == 1) {
    if (geofs.animation.values.throttle >= 0.1) {
      geofs.animation.values.isRetard = 1;
    }
    else {
      geofs.animation.values.isRetard = 0;
    }
  }
  else {
    geofs.animation.values.isRetard = 0;
  }
}

function doRadioAltCall() {
  if (isApprConfig) {
    if (geofs.animation.values.haglFeet <= 1000 + restingPoint && geofs.animation.values.haglFeet >= 900 + restingPoint) {
      geofs.animation.values.gpws1000 = 1;
    }
    else {
      geofs.animation.values.gpws1000 = 0;
    }
    if (geofs.animation.values.haglFeet <= 500 + restingPoint && geofs.animation.values.haglFeet >= 400 + restingPoint) {
      geofs.animation.values.gpws500 = 1;
    }
    else {
      geofs.animation.values.gpws500 = 0;
    }
    if (geofs.animation.values.haglFeet <= 400 + restingPoint && geofs.animation.values.haglFeet >= 300 + restingPoint) {
      geofs.animation.values.gpws400 = 1;
    }
    else {
      geofs.animation.values.gpws400 = 0;
    }
    if (geofs.animation.values.haglFeet <= 300 + restingPoint && geofs.animation.values.haglFeet >= 200 + restingPoint) {
      geofs.animation.values.gpws300 = 1;
    }
    else {
      geofs.animation.values.gpws300 = 0;
    }
    if (geofs.animation.values.haglFeet <= 200 + restingPoint && geofs.animation.values.haglFeet >= 100 + restingPoint) {
      geofs.animation.values.gpws200 = 1;
    }
    else {
      geofs.animation.values.gpws200 = 0;
    }
    if (geofs.animation.values.haglFeet <= 100 + restingPoint && geofs.animation.values.haglFeet >= 50 + restingPoint) {
      geofs.animation.values.gpws100 = 1;
    }
    else {
      geofs.animation.values.gpws100 = 0;
    }
    if (geofs.animation.values.haglFeet <= 50 + restingPoint && geofs.animation.values.haglFeet >= 40 + restingPoint) {
      geofs.animation.values.gpws50 = 1;
    }
    else {
      geofs.animation.values.gpws50 = 0;
    }
    if (geofs.animation.values.haglFeet <= 40 + restingPoint && geofs.animation.values.haglFeet >= 30 + restingPoint) {
      geofs.animation.values.gpws40 = 1;
    }
    else {
      geofs.animation.values.gpws40 = 0;
    }
    if (geofs.animation.values.haglFeet <= 30 + restingPoint && geofs.animation.values.haglFeet >= 20 + restingPoint) {
      geofs.animation.values.gpws30 = 1;
    }
    else {
      geofs.animation.values.gpws30 = 0;
    }
    if (geofs.animation.values.haglFeet <= 20 + restingPoint && geofs.animation.values.haglFeet >= 10 + restingPoint) {
      geofs.animation.values.gpws20 = 1;
    }
    else {
      geofs.animation.values.gpws20 = 0;
    }
    if (geofs.animation.values.haglFeet <= 10 + restingPoint && geofs.animation.values.haglFeet >= 5 + restingPoint) {
      geofs.animation.values.gpws10 = 1;
    }
    else {
      geofs.animation.values.gpws10 = 0;
    }
  }
  else {
    geofs.animation.values.gpws1000 = 0;
    geofs.animation.values.gpws500 = 0;
    geofs.animation.values.gpws400 = 0;
    geofs.animation.values.gpws300 = 0;
    geofs.animation.values.gpws200 = 0;
    geofs.animation.values.gpws100 = 0;
    geofs.animation.values.gpws50 = 0;
    geofs.animation.values.gpws40 = 0;
    geofs.animation.values.gpws30 = 0;
    geofs.animation.values.gpws20 = 0;
    geofs.animation.values.gpws10 = 0;
  }
}

function resetLift2() {
  geofs.animation.values.liftLeftWing = (-geofs.aircraft.instance.parts.wingleft.lift / 200000) + (geofs.animation.values.accZ) / 20;
  geofs.animation.values.liftRightWing = (-geofs.aircraft.instance.parts.wingright.lift / 200000) + (geofs.animation.values.accZ) / 20;
};

setInterval(function() {
  getTrafficProximity();
  getFlapChange();
  getWindShear();
  testForApproach();
  testTerrainorAppr();
  resetLift2();
  getRetard();
  doRadioAltCall()
})
//assign alarms and sound fx

geofs.aircraft.instance.definition.sounds[11] = {};
geofs.aircraft.instance.definition.sounds[11].id = "flapssound"
geofs.aircraft.instance.definition.sounds[11].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/777flap.mp3"
geofs.aircraft.instance.definition.sounds[11].effects = { "start": { "value": "flapschange" } }

geofs.aircraft.instance.definition.sounds[12] = {};
geofs.aircraft.instance.definition.sounds[12].id = "landinggearwarn"
geofs.aircraft.instance.definition.sounds[12].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/tlg.mp3"
geofs.aircraft.instance.definition.sounds[12].effects = { "start": { "value": "isGearWarn" } }

geofs.aircraft.instance.definition.sounds[13] = {};
geofs.aircraft.instance.definition.sounds[13].id = "flapswarn"
geofs.aircraft.instance.definition.sounds[13].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/tlf.mp3"
geofs.aircraft.instance.definition.sounds[13].effects = { "start": { "value": "isFlapsWarn" } }

geofs.aircraft.instance.definition.sounds[14] = {};
geofs.aircraft.instance.definition.sounds[14].id = "terrainwarn"
geofs.aircraft.instance.definition.sounds[14].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/tlt.mp3"
geofs.aircraft.instance.definition.sounds[14].effects = { "start": { "value": "isTerrainWarn" } }

geofs.aircraft.instance.definition.sounds[15] = {};
geofs.aircraft.instance.definition.sounds[15].id = "pullwarn"
geofs.aircraft.instance.definition.sounds[15].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/pullup.mp3"
geofs.aircraft.instance.definition.sounds[15].effects = { "start": { "value": "isPullupWarn" } }

geofs.aircraft.instance.definition.sounds[16] = {};
geofs.aircraft.instance.definition.sounds[16].id = "1000"
geofs.aircraft.instance.definition.sounds[16].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/1000ab.mp3"
geofs.aircraft.instance.definition.sounds[16].effects = { "start": { "value": "gpws1000" } }

geofs.aircraft.instance.definition.sounds[17] = {};
geofs.aircraft.instance.definition.sounds[17].id = "500"
geofs.aircraft.instance.definition.sounds[17].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/500ab.mp3"
geofs.aircraft.instance.definition.sounds[17].effects = { "start": { "value": "gpws500" } }

geofs.aircraft.instance.definition.sounds[18] = {};
geofs.aircraft.instance.definition.sounds[18].id = "400"
geofs.aircraft.instance.definition.sounds[18].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/400ab.mp3"
geofs.aircraft.instance.definition.sounds[18].effects = { "start": { "value": "gpws400" } }

geofs.aircraft.instance.definition.sounds[19] = {};
geofs.aircraft.instance.definition.sounds[19].id = "300"
geofs.aircraft.instance.definition.sounds[19].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/300ab.mp3"
geofs.aircraft.instance.definition.sounds[19].effects = { "start": { "value": "gpws300" } }

geofs.aircraft.instance.definition.sounds[20] = {};
geofs.aircraft.instance.definition.sounds[20].id = "200"
geofs.aircraft.instance.definition.sounds[20].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/200ab.mp3"
geofs.aircraft.instance.definition.sounds[20].effects = { "start": { "value": "gpws200" } }

geofs.aircraft.instance.definition.sounds[21] = {};
geofs.aircraft.instance.definition.sounds[21].id = "100"
geofs.aircraft.instance.definition.sounds[21].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/100abtrue.mp3"
geofs.aircraft.instance.definition.sounds[21].effects = { "start": { "value": "gpws100" } }

geofs.aircraft.instance.definition.sounds[22] = {};
geofs.aircraft.instance.definition.sounds[22].id = "50"
geofs.aircraft.instance.definition.sounds[22].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/50ab.mp3"
geofs.aircraft.instance.definition.sounds[22].effects = { "start": { "value": "gpws50" } }

geofs.aircraft.instance.definition.sounds[23] = {};
geofs.aircraft.instance.definition.sounds[23].id = "40"
geofs.aircraft.instance.definition.sounds[23].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/40ab.mp3"
geofs.aircraft.instance.definition.sounds[23].effects = { "start": { "value": "gpws40" } }

geofs.aircraft.instance.definition.sounds[24] = {};
geofs.aircraft.instance.definition.sounds[24].id = "30"
geofs.aircraft.instance.definition.sounds[24].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/30ab.mp3"
geofs.aircraft.instance.definition.sounds[24].effects = { "start": { "value": "gpws30" } }

geofs.aircraft.instance.definition.sounds[25] = {};
geofs.aircraft.instance.definition.sounds[25].id = "20"
geofs.aircraft.instance.definition.sounds[25].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/20ab.mp3"
geofs.aircraft.instance.definition.sounds[25].effects = { "start": { "value": "gpws20" } }

geofs.aircraft.instance.definition.sounds[26] = {};
geofs.aircraft.instance.definition.sounds[26].id = "10"
geofs.aircraft.instance.definition.sounds[26].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/10ab.mp3"
geofs.aircraft.instance.definition.sounds[26].effects = { "start": { "value": "gpws10" } }

geofs.aircraft.instance.definition.sounds[27] = {};
geofs.aircraft.instance.definition.sounds[27].id = "TCAS"
geofs.aircraft.instance.definition.sounds[27].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/traffic.mp3"
geofs.aircraft.instance.definition.sounds[27].effects = { "start": { "value": "isTCAS" } }

geofs.aircraft.instance.definition.sounds[28] = {};
geofs.aircraft.instance.definition.sounds[28].id = "climb"
geofs.aircraft.instance.definition.sounds[28].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/climb.mp3"
geofs.aircraft.instance.definition.sounds[28].effects = { "start": { "value": "isTCASClimb" } }

geofs.aircraft.instance.definition.sounds[29] = {};
geofs.aircraft.instance.definition.sounds[29].id = "descend"
geofs.aircraft.instance.definition.sounds[29].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/descend.mp3"
geofs.aircraft.instance.definition.sounds[29].effects = { "start": { "value": "isTCASDescend" } }

geofs.aircraft.instance.definition.sounds[30] = {};
geofs.aircraft.instance.definition.sounds[30].id = "clear"
geofs.aircraft.instance.definition.sounds[30].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/clear.mp3"
geofs.aircraft.instance.definition.sounds[30].effects = { "start": { "value": "isTCASClear" } }

geofs.aircraft.instance.definition.sounds[31] = {};
geofs.aircraft.instance.definition.sounds[31].id = "retard"
geofs.aircraft.instance.definition.sounds[31].file = "https://138772948-227015667470610340.preview.editmysite.com/uploads/1/3/8/7/138772948/retard.mp3"
geofs.aircraft.instance.definition.sounds[31].effects = { "start": { "value": "isRetard" } }

audio.init(geofs.aircraft.instance.definition.sounds);
geofs.aircraft.instance.definition.sounds[0].effects.volume.ratio = 100
geofs.aircraft.instance.definition.sounds[1].effects.volume.ratio = 100
geofs.aircraft.instance.definition.sounds[2].effects.volume.ratio = 100
geofs.aircraft.instance.definition.sounds[3].effects.volume.ratio = 100
geofs.aircraft.instance.definition.sounds[7].effects.volume.ratio = 100
geofs.aircraft.instance.definition.sounds[8].effects.volume.ratio = 100
geofs.aircraft.instance.definition.sounds[9].effects.volume.ratio = 100

//Terrain Radar
let terrainPoints = [];
function getRadar(resolution) {
  if (terrainPoints.length > resolution) {
  terrainPoints = [];
  }
  for (let i = 0; i < 500; i++) {
    let distance = i/8 % 3;
    let directionStart = geofs.animation.values.heading - 90
    let direction = directionStart - i /5
    let x1 = geofs.aircraft.instance.llaLocation[0];
    let y1 = geofs.aircraft.instance.llaLocation[1];
    let x2 = distance*Math.sin(Math.PI*direction/180);
    let y2 = distance*Math.cos(Math.PI*direction/180);
    terrainPoints.push([distance*100, Math.PI*((i/5  - 225)/180), geofs.getGroundAltitude(x1+x2,y1+y2).location[2]]);
  }
  
}

let toggleRadar = 0

function radar(){
if (toggleRadar == 0){
  toggleRadar = 1;
}
  else{
    toggleRadar = 0;
  }
}


//ILS program

function getDistance(lat1, lon1, lat2, lon2) {
  var R = 6371;
  var dLat = deg2rad(lat2 - lat1);
  var dLon = deg2rad(lon2 - lon1);
  var a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) *
    Math.sin(dLon / 2) * Math.sin(dLon / 2)
    ;
  var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  var d = R * c;
  return d;
}

function deg2rad(deg) {
  return deg * (Math.PI / 180)
}


function computeSlopeDeviation(ae, alt, alg, lt, lg, a) {
  let gradient = 0.05;
  let distance = getDistance(alt, alg, lt, lg) * 1000;
  let idealAltAtPos = ae + gradient * distance;
  let deviation = idealAltAtPos - a;
  return deviation;
}

//runway side detection from autoland 1.0

function getRwHeading() {
  let defaultRunway = runway.heading;
  let aircraftHeading = geofs.animation.values.heading360

  if (aircraftHeading >= defaultRunway + 90 || aircraftHeading <= defaultRunway - 90) {
    let sideHeading = runway.heading + 180;
    let sideHeadingFixed = sideHeading % 360;
    return sideHeadingFixed;
  }
  else {
    return defaultRunway;
  }
}

function getRwThreshold() {
  let defaultRunway = runway.heading;
  let aircraftHeading = geofs.animation.values.heading;

  if (aircraftHeading >= defaultRunway + 90 || aircraftHeading <= defaultRunway - 90) {

    let x1 = runway.location[1];
    let y1 = runway.location[0];
    let x2 = runway.lengthInLla[1];
    let y2 = runway.lengthInLla[0];
    let runwayThresholdX = x1 + x2;
    let runwayThresholdY = y1 + y2;
    let runwayThreshold = [runwayThresholdY, runwayThresholdX, 0];
    return runwayThreshold;
  }

  else {
    let runwayThreshold = runway.location
    return runwayThreshold;
  }
}




function radians(n) {
  return n * (Math.PI / 180);
}
function degrees(n) {
  return n * (180 / Math.PI);
}

//yes i know ive defined those functions like 3 times now but whatever lol


//main function to find the direction to the airport. a perfect localizer capture will mean that the runway heading - function output = 0.
function getBearing(a, b, c, d) {
  startLat = radians(c);
  startLong = radians(d);
  endLat = radians(a);
  endLong = radians(b);

  let dLong = endLong - startLong;

  let dPhi = Math.log(Math.tan(endLat / 2.0 + Math.PI / 4.0) / Math.tan(startLat / 2.0 + Math.PI / 4.0));
  if (Math.abs(dLong) > Math.PI) {
    if (dLong > 0.0)
      dLong = -(2.0 * Math.PI - dLong);
    else
      dLong = (2.0 * Math.PI + dLong);
  }

  return (degrees(Math.atan2(dLong, dPhi)) + 360.0) % 360.0;
}

function computeLocDeviation(alt, alg, lt, lg) {
  return getRwHeading() - getBearing(alt, alg, lt, lg);
}

function getNearestRunway() {
  return Object.values(geofs.runways.nearRunways)[minKey];
}

let runway = ""

function displayDeviations() {
  a = getRwThreshold()
  b = geofs.aircraft.instance.llaLocation
  locdev = clamp(-computeLocDeviation(a[0], a[1], b[0], b[1]) * 20, -250, 250);
  gsdev = clamp(3 * computeSlopeDeviation(Object.values(geofs.api.flatRunwayTerrainProviderInstance.regions)[0].referenceElevation, a[0], a[1], b[0], b[1], (geofs.animation.values.altitudeMeters - 4)), -500, 500);
}
//ils display
let ilshead = 0; // will set this to geofs.animation.values.heading360 later
let locdev = 0;
let gsdev = 0;
let traffic = Object.values(multiplayer.visibleUsers)

class ILSsim {
  constructor(resX, resY, sizeX, sizeY) {
    // IMP VALUES ! NO CHANGE !!
    this.Values = {};
    this.Values.LocDev = 0;
    this.Values.GSDev = 0;
    this.Values.Heading = 0;

    // Everything Else LOL
    this.VisibilityToggleButton;
    this.Display = {};
    this.Display.Element;
    this.Display.Context;
    this.Display.Width = resX;
    this.Display.Height = resY;
    this.Display.SizeWidth = sizeX;
    this.Display.SizeHeight = sizeY;
    this.Events = [];
  }
  // Implement Show/Hide Canvas
  AssignVisibilityToggleButton(element) {
    this.VisibilityToggleButton = element;
    let self = this;
    this.VisibilityToggleButton.onclick = function() {
      if (this.innerText == "show") {
        self.Display.Element.style.visibility = "visible";
        this.innerText = "hide";
      } else {
        self.Display.Element.style.visibility = "hidden";
        this.innerText = "show";
      }
    };
  }
  MakeLine(color, x1, y1, x2, y2) {
    this.Display.Context.beginPath();
    this.Display.Context.strokeStyle = color;
    this.Display.Context.moveTo(x1, y1);
    this.Display.Context.lineTo(x2, y2);
    this.Display.Context.stroke();
  }
  MakeText(text, color, x, y, font) {
    this.Display.Context.beginPath();
    let prevColor = this.Display.Context.fillStyle;
    let prevFont = this.Display.Context.font;
    this.Display.Context.beginPath();
    this.Display.Context.fillStyle = color;
    if (font) {
      this.Display.Context.font = font;
    }
    this.Display.Context.fillText(text, x, y);
    this.Display.Context.fillStyle = prevColor;
    this.Display.Context.font = prevFont;
  }
  // Non-Interactive Rectangle Making.
  MakeRect(fill, color, x, y, width, height) {
    this.Display.Context.beginPath();
    this.Display.Context.strokeStyle = color;
    this.Display.Context.rect(x, y, width, height);
    this.Display.Context.stroke();
    this.Display.Context.fillStyle = fill;
    this.Display.Context.fill();
  }
  // Interactive Rectangle Making.
  MakePolygon(points, fillColor, outlineColor, onclick) {
    if (onclick) {
      this.AddEventToCanvas(points, onclick);
    }
    this.Display.Context.beginPath();
    this.Display.Context.moveTo(points[0][0], points[0][1]);
    points = points.slice(1);
    let a;
    for (a of points) {
      let x = a[0];
      let y = a[1];
      this.Display.Context.lineTo(x, y);
    }
    this.Display.Context.fillStyle = fillColor;
    this.Display.Context.fill();
    this.Display.Context.strokeStyle = outlineColor;
    this.Display.Context.stroke();
  }
  MakeCircle(fillColor, strokeColor, x, y, r, startAngle, endAngle, antiClockwise) {
    this.Display.Context.beginPath();
    if (startAngle) {
      this.Display.Context.arc(x, y, startAngle, endAngle, antiClockwise);
    } else {
      this.Display.Context.arc(x, y, r, 0, 2 * Math.PI);
    }
    this.Display.Context.strokeStyle = strokeColor;
    this.Display.Context.stroke();
    this.Display.Context.fillStyle = fillColor;
    this.Display.Context.fill();
  }
  MakeRoundSlider(x, y, r, value, color1, color2, color3, color4, color5, color6, mouseMoveFunction, mouseUpFunction) {
    let extractedValue = this.Values[value];
    let direction = 360 / 100 * extractedValue;
    direction -= 90;
    this.MakeCircle(color2, color1, x, y, r);
    this.MakeCircle(color4, color3, x, y, r * 0.5);
    this.MakePolygon([
      [x + (Math.cos(A2R(direction + 90)) * -1), y + (Math.sin(A2R(direction + 90)) * -1)],
      [x + (Math.cos(A2R(direction + 90)) * 1), y + (Math.sin(A2R(direction + 90)) * 1)],
      [x + (Math.cos(A2R(direction + 90)) * 1) + (Math.cos(A2R(direction)) * r), y + (Math.sin(A2R(direction + 90)) * 1) + (Math.sin(A2R(direction)) * r)],
      [x + (Math.cos(A2R(direction + 90)) * -1) + (Math.cos(A2R(direction)) * r), y + (Math.sin(A2R(direction + 90)) * -1) + (Math.sin(A2R(direction)) * r)]
    ], color6, color5, function(a) {
      a.path[0].onmouseup = function(b) {
        mouseUpFunction(b);
        b.path[0].onmousemove = undefined;
      };
      a.path[0].onmousemove = function(b) {
        mouseMoveFunction(b);
      };
    });
  }
  AddEventToCanvas(points, func) {
    let newObj = { "points": points, "func": func };
    this.Events[this.Events.length] = newObj;
  }
  RemoveEventFromCanvas(event) {
    let index = this.Events.indexOf(event);
    if (index > -1) {
      this.Events.splice(index, 1);
    }
  }
  ResetEvents() {
    this.Events.length = 0;
  }
  SetupEventHandler() {
    let self = this;
    this.Display.Element.onmousedown = function(event) {
      let rect = event.target.getBoundingClientRect();
      let xRelation = self.Display.Width / self.Display.SizeWidth.slice(0, self.Display.SizeWidth.length - 2);
      let yRelation = self.Display.Height / self.Display.SizeHeight.slice(0, self.Display.SizeHeight.length - 2);
      let x = event.clientX - rect.left;
      let y = event.clientY - rect.top;
      x *= xRelation;
      y *= yRelation;
      console.log("X: " + x + "\nY: " + y);
      console.log(event);
      let a;
      for (a of self.Events) {
        let func = a.func;
        let vs = a.points;
        if (inside([x, y], vs)) {
          func(event);
          self.ResetEvents();
        }
      }
      self.rDraw();
    };
  }
  SetupCanvas() {
    this.Display.Element = document.createElement("canvas");
    this.Display.Element.width = this.Display.Width;
    this.Display.Element.height = this.Display.Height;
    this.Display.Element.style.width = this.Display.SizeWidth;
    this.Display.Element.style.height = this.Display.SizeHeight;
    this.Display.Element.style.position = "absolute";
    this.Display.Element.style.left = "25%";
    this.Display.Element.style.top = "25%";
    this.Display.Element.style.transform = "translate(-50%, -50%)";
    this.Display.Element.style.imageRendering = "pixelated";
    document.body.appendChild(this.Display.Element);
    this.Display.Context = this.Display.Element.getContext("2d");
    this.Display.Context.lineWidth = 10;
  }
  rDraw() {
    let w = this.Display.Width;
    let h = this.Display.Height;
    this.Display.Context.clearRect(0, 0, w, h)
    this.Draw();
  }
  Draw() {
    function getDeviation() {
      let b = ilshead
      let a = Math.PI * (b / 180);
      let d = locdev;
      let c1 = Math.sin(Math.PI * (heading / 180)) * 100;
      let c2 = Math.cos(Math.PI * (heading / 180)) * 100;
      let c3 = Math.PI * (b + 90) / 180;
      let origin = [w - w / 1.9, h - h / 2];
      let x1 = origin[0] + d * Math.sin(c3);
      let y1 = origin[1] - d * Math.cos(c3);
      let x2 = origin[0] + c1 + (d * Math.sin(c3));
      let y2 = origin[1] - c2 - (d * Math.cos(c3));
      // console.log([x1, y1, x2, y2, c3]); //For debugging
      return [x1, y1, x2, y2];
    }

    function getTrafficIndicator(direction, distance) {
      let directionRad = Math.PI * (direction / 180);
      let origin = [w - w / 1.9, h - h / 2];
      let x1 = origin[0] + distance * Math.sin(directionRad);
      let y1 = origin[1] + distance * Math.cos(directionRad);
      return [x1, y1];
    }

    let heading = ilshead;
    let w = this.Display.Width;
    let h = this.Display.Height;
    this.MakeRect("black", "black", 0, 0, w, h);
    this.MakeCircle("white", "white", w - w / 1.9, h - h / 2, 300);
    this.MakeCircle("black", "white", w - w / 1.9, h - h / 2, 295);
    //heading lines
    this.MakeLine("#e600ff", w - w / 1.9, h - h / 2, w - w / 1.9 + Math.sin(Math.PI * (heading / 180)) * 300, h - h / 2 - Math.cos(Math.PI * (heading / 180)) * 300);
    this.MakeLine("#e600ff", w - w / 1.9, h - h / 2, w - w / 1.9 - Math.sin(Math.PI * (heading / 180)) * 300, h - h / 2 + Math.cos(Math.PI * (heading / 180)) * 300);
    this.MakeCircle("black", "black", w - w / 1.9, h - h / 2, 100);
    //terrain radar
    if (toggleRadar == 1) {
    terrainPoints.forEach(function(e){
      let elevation = geofs.animation.values.altitudeMeters;
      if (e[2] < elevation - 1000){
      display.MakeCircle("green", "#ffffff00", w - w / 1.9 +e[0]*Math.sin(e[1]), h - h / 2 + e[0]*Math.cos(e[1]), Math.abs(1+e[0]/20));
      }
      if (e[2] > elevation - 1000 && e[2] < elevation){
        display.MakeCircle("yellow", "#ffffff00", w - w / 1.9 +e[0]*Math.sin(e[1]), h - h / 2 + e[0]*Math.cos(e[1]), Math.abs(1+e[0]/20));
      }
      if (e[2] > elevation) {
        display.MakeCircle("red", "#ffffff00", w - w / 1.9 +e[0]*Math.sin(e[1]), h - h / 2 + e[0]*Math.cos(e[1]), Math.abs(1+e[0]/20));
      }
    })
    }
    //traffic
    traffic.forEach(function(e) {
      display.MakeCircle("black", "blue", getTrafficIndicator(geofs.animation.values.heading+getBearing(e.referencePoint.lla[0], e.referencePoint.lla[1], geofs.aircraft.instance.llaLocation[0], geofs.aircraft.instance.llaLocation[1]), e.distance / 100)[0], getTrafficIndicator(geofs.animation.values.heading+getBearing(e.referencePoint.lla[0], e.referencePoint.lla[1], geofs.aircraft.instance.llaLocation[0], geofs.aircraft.instance.llaLocation[1]), e.distance / 100)[1], 5)
    })
    //aircraft indicator
    this.MakeLine("yellow", w - w / 1.9, h - h / 2 + 20, w - w / 1.9 + 60, h - h / 2 + 20);
    this.MakeLine("yellow", w - w / 1.9, h - h / 2 + 20, w - w / 1.9 - 60, h - h / 2 + 20);
    this.MakeLine("yellow", w - w / 1.9, h - h / 2, w - w / 1.9, h - h / 2 + 100);
    this.MakeLine("yellow", w - w / 1.9, h - h / 2, w - w / 1.9, h - h / 2 - 20);
    this.MakeLine("yellow", w - w / 1.9, h - h / 2 + 75, w - w / 1.9 + 25, h - h / 2 + 75);
    this.MakeLine("yellow", w - w / 1.9, h - h / 2 + 75, w - w / 1.9 - 25, h - h / 2 + 75);
    // gs indicator
    this.MakeCircle("black", "#e600ff", w - w / 15, (h - h / 2 + 7) - gsdev, 20);
    //gs deviation markers
    this.MakeRect("yellow", "yellow", w - w / 10, h - h / 2, w / 15, h / 100);
    this.MakeCircle("white", "white", w - w / 15, h - h / 1.5, 15);
    this.MakeCircle("black", "white", w - w / 15, h - h / 1.5, 8);
    this.MakeCircle("white", "white", w - w / 15, h - h / 1.2, 15);
    this.MakeCircle("black", "white", w - w / 15, h - h / 1.2, 8);
    this.MakeCircle("white", "white", w - w / 15, h - h / 3.25, 15);
    this.MakeCircle("black", "white", w - w / 15, h - h / 3.25, 8);
    this.MakeCircle("white", "white", w - w / 15, h - h / 6.5, 15);
    this.MakeCircle("black", "white", w - w / 15, h - h / 6.5, 8);
    //loc deviation markers
    this.MakeCircle("black", "white", w - w / 1.9 - Math.cos(Math.PI * (heading / 180)) * 75, h - h / 2 - Math.sin(Math.PI * (heading / 180)) * 75, 8);
    this.MakeCircle("black", "white", w - w / 1.9 - Math.cos(Math.PI * (heading / 180)) * 200, h - h / 2 - Math.sin(Math.PI * (heading / 180)) * 200, 8);
    this.MakeCircle("black", "white", w - w / 1.9 + Math.cos(Math.PI * (heading / 180)) * 75, h - h / 2 + Math.sin(Math.PI * (heading / 180)) * 75, 8);
    this.MakeCircle("black", "white", w - w / 1.9 + Math.cos(Math.PI * (heading / 180)) * 200, h - h / 2 + Math.sin(Math.PI * (heading / 180)) * 200, 8);
    this.MakeLine("#e600ff", getDeviation()[0], getDeviation()[1], getDeviation()[2], getDeviation()[3]);
    
  };

}
let display
let rwDistances = [];
let minKey = 0;

function ilsIntervalStart() {
ilsInterval = setInterval(function() {
  rwDistances = []
  Object.values(geofs.runways.nearRunways).forEach(function(e){
rwDistances.push(getDistance(e.location[0], e.location[1], geofs.aircraft.instance.llaLocation[0], geofs.aircraft.instance.llaLocation[1]));
})
  rwDistances.forEach(function(e, i){
    if (e == Math.min(...rwDistances)) {
      minKey = i;
    }
  })
      ;
  traffic = Object.values(multiplayer.visibleUsers);
  runway = getNearestRunway();
  ilshead = getRwHeading() - geofs.animation.values.heading360;
  displayDeviations()
  display.rDraw()
}, 200)
}
let terrainInterval = setInterval(function(){
  getRadar(100)
}, 1000)

let hide = false
function togglePanel(){
  if (!hide){
 display = new ILSsim(1000, 1000, "250px", "250px");
display.SetupCanvas();
display.SetupEventHandler();
display.Draw();
    ilsIntervalStart()
  hide = true;
  }
  else {
    destroyDisplays()
    hide = false;
  }
};

let array = []

function destroyDisplays() {
  array = []
  Object.values(document.getElementsByTagName("canvas")).forEach(function(e){if (e.width == 1000) array.push(e)})
  array.forEach(function(e){e.remove()})
}

//temporary menu for user control
// Panel Code
let a320panel = document.createElement("div");
a320panel.innerHTML = '<ul class="geofs-list geofs-toggle-panel geofs-autoland-list geofs-preferences" data-noblur="true" data-onshow="{geofs.initializePreferencesPanel()}" data-onhide="{geofs.savePreferencesPanel()}"><style>#MainDIV {position: absolute;left: 0px;top: 0px;background-color: white;border: 5px solid #000000;text-align: center;padding: 0px 10px 10px 10px;}#DIVtitle {color: black;font-family: monospace;font-weight: bold;font-size: 20px;}p {color: black;font-family: monospace;font-weight: bold;}.button {display: inline-block;padding: 3px 24px;font-size: 15px;cursor: pointer;text-align: center;text-decoration: none;outline: none;color: black;background-color: #ffc107;border: none;border-radius: 1px;box-shadow: 0 0px #999;}.button2 {display: inline-block}.button:hover {background-color: #536dfe}.button:active {opacity: 0.6;}.button3 {display: inline-block;padding: 3px 24px;font-size: 15px;cursor: pointer;text-align: center;text-decoration: none;outline: none;color: #fff;background-color: #536dfe;border: none;border-radius: 1px;box-shadow: 0 0px #999;}.button4 {display: inline-block;padding: 3px 24px;font-size: 15px;cursor: pointer;text-align: center;text-decoration: none;outline: none;color: #fff;background-color: red;border: none;border-radius: 1px;box-shadow: 0 0px #999;}</style><div id="MainDIV"><p id="DIVtitle">A321Neo User Interface</p><p>Engine Controls:</p><button onclick="startEngine(`left`)" class="button", id="leftstart">Start Left Engine</button><button onclick="startEngine(`right`)" class = "button" id="rightstart">Start Right Engine</button><button onclick="stopEngines()" class = "button">Stop Engines</button><button class = "button" onclick = "togglePanel()">Toggle ILS panel</button></div></ul>'

let sidePanel = document.getElementsByClassName("geofs-ui-left")[0]
document.getElementsByClassName("geofs-ui-left")[0].appendChild(a320panel)

// Toggle Button Code
let buttonDiv = document.createElement("div");
buttonDiv.innerHTML = '<button class="mdl-button mdl-js-button geofs-f-standard-ui geofs-mediumScreenOnly" data-toggle-panel=".geofs-autoland-list" data-tooltip-classname="mdl-tooltip--top" id="landButton" tabindex="0" data-upgraded=",MaterialButton">A321Neo</button>'
document.body.appendChild(buttonDiv);
document.getElementsByClassName("geofs-ui-bottom")[0].appendChild(buttonDiv);
let element = document.getElementById("landButton");
document.getElementsByClassName("geofs-ui-bottom")[0].insertBefore(element, buttonDiv);

