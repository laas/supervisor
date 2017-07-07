rosservice call /pdg/remove_from_hand "{objectId: 'BLUE_CUBE1'}"
rosservice call /pdg/remove_from_hand "{objectId: 'BLUE_CUBE2'}"
rosservice call /pdg/remove_from_hand "{objectId: 'BLUE_CUBE3'}"
rosservice call /pdg/remove_from_hand "{objectId: 'RED_CUBE1'}"
rosservice call /pdg/remove_from_hand "{objectId: 'RED_CUBE2'}"
rosservice call /pdg/remove_from_hand "{objectId: 'RED_CUBE3'}"
rosservice call /pdg/remove_from_hand "{objectId: 'GREEN_CUBE1'}"
rosservice call /pdg/remove_from_hand "{objectId: 'GREEN_CUBE2'}"
rosservice call /pdg/remove_from_hand "{objectId: 'GREEN_CUBE3'}"
rosservice call /pdg/remove_from_hand "{objectId: 'RED_TAPE1'}"
rosservice call /pdg/remove_from_hand "{objectId: 'RED_TAPE2'}"



rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosservice call /toaster_simu/set_entity_pose "{id: 'BLACK_CUBE', ownerId: '', type: 'object', pose:
{position:
  {x: 3.5,
  y: 6.4,
  z: 0.76},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"


rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 8.8,
  y: 10.6,
  z: 0.73},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 3.5,
  y: 6.4,
  z: 0.76},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE3', ownerId: '', type: 'object', pose:
{position:
  {x: 8.8,
  y: 10.6,
  z: 0.61},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 8.8,
  y: 10.6,
  z: 0.85},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 3.5,
  y: 6.4,
  z: 0.88},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_CUBE3', name: 'GREEN_CUBE3', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE3', ownerId: '', type: 'object', pose:
{position:
  {x: 8.8,
  y: 10.6,
  z: 0.67},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 3.5,
  y: 6.4,
  z: 0.94},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 8.8,
  y: 10.6,
  z: 0.79},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE3', ownerId: '', type: 'object', pose:
{position:
  {x: 3.5,
  y: 6.4,
  z: 0.82},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'RED_TAPE1', ownerId: '', type: 'object', pose:
{position:
  {x: 3.3,
  y: 6.2,
  z: 0.79},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'RED_TAPE2', ownerId: '', type: 'object', pose:
{position:
  {x: 6.0,
  y: 11.5,
  z: 0.8},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE3', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'cube', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE1', targetId: 'tape', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE2', targetId: 'tape', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA1', targetId: 'area', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA2', targetId: 'area', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'AGENTX', targetId: 'X', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'AGENTX2', targetId: 'X', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'human', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PR2_ROBOT', targetId: 'robot', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE3', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE1', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE2', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX1', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX2', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'BLUE_BOX', targetId: 'blue', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_BOX', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'HERAKLES_HUMAN1', facts: [
    {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'GREEN_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'BLUE_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'RED_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'BLUE_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'RED_CUBE1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'GREEN_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'RED_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'GREEN_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'BLUE_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'RED_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'BLUE_CUBE2', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'RED_CUBE1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'GREEN_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isOn', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'RED_CUBE3', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
, {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLACK_CUBE', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'value', propertyType: 'state', subProperty: '', subjectId: 'FTRUE', targetId: 'true', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'value', propertyType: 'state', subProperty: '', subjectId: 'FFALSE', targetId: 'false', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'HERAKLES_HUMAN1', facts: [
    {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE3', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLUE_BOX', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX1', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE1', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA1', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA2', targetId: 'PR2_ROBOT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE3', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE3', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'GREEN_BOX', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_BOX2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'RED_TAPE2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA1', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'isReachableBy', propertyType: 'state', subProperty: '', subjectId: 'SCAN_AREA2', targetId: 'HERAKLES_HUMAN1', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 1.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

