rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 3.6,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.5,
  y: 3.7,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"
  
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 0.0,
  y: 0.0,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"
 

rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.5,
  y: 3.5,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 5.1,
  y: 4.5,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 5.2,
  y: 4.6,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/set_entity_pose "{id: 'STICK', ownerId: '', type: 'object', pose:
{position:
  {x: 5.2,
  y: 4.4,
  z: 0.7},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'STICK', targetId: 'STICK', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_1', targetId: 'AREA', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_2', targetId: 'AREA', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'AGENTX', targetId: 'X', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'AGENTX2', targetId: 'X', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'human', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'PR2_ROBOT', targetId: 'robot', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'type', propertyType: 'state', subProperty: '', subjectId: 'OMNI', targetId: 'OMNI', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE1', targetId: 'RED_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'RED_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE1', targetId: 'GREEN_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'GREEN_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE1', targetId: 'BLUE_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'BLUE_CUBE2', targetId: 'BLUE_CUBE', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'STICK', targetId: 'STICK', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_1', targetId: 'PLACEMAT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'genericName', propertyType: 'state', subProperty: '', subjectId: 'PLACEMAT_2', targetId: 'PLACEMAT', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'value', propertyType: 'state', subProperty: '', subjectId: 'FTRUE', targetId: 'true', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'value', propertyType: 'state', subProperty: '', subjectId: 'FFALSE', targetId: 'false', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"
