rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false, toasterSimuHuman: true, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true, arObject: false}"

rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosservice call /toaster_simu/add_entity "{id: 'TABLE_4', name: 'TABLE_4', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'TABLE_4', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 4.1,
  z: 0.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

  rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_ROBOT', name: 'PLACEMAT_ROBOT', type: 'object', ownerId: ''}"
  rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_ROBOT', ownerId: '', type: 'object', pose:
  {position:
    {x: 4.7,
    y: 3.8,
    z: 0.73},
  orientation:
    {x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0}}}"

  rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_HUMAN1', name: 'PLACEMAT_HUMAN1', type: 'object', ownerId: ''}"
  rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_HUMAN1', ownerId: '', type: 'object', pose:
  {position:
    {x: 5.05,
    y: 4.1,
    z: 0.73},
  orientation:
    {x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0}}}"

  rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_HUMAN2', name: 'PLACEMAT_HUMAN2', type: 'object', ownerId: ''}"
  rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_HUMAN2', ownerId: '', type: 'object', pose:
  {position:
    {x: 4.8,
    y: 4.2,
    z: 0.73},
  orientation:
    {x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0}}}"

  rosservice call /toaster_simu/add_entity "{id: 'IKEA_SHELF_LIGHT_1', name: 'IKEA_SHELF_LIGHT_1', type: 'object', ownerId: ''}"
  rosservice call /toaster_simu/set_entity_pose "{id: 'IKEA_SHELF_LIGHT_1', ownerId: '', type: 'object', pose:
  {position:
    {x: 4.0,
    y: 3.3,
    z: 0.0},
  orientation:
    {x: 0.0,
    y: 0.0,
    z: 0.7,
    w: 0.7}}}"

  rosservice call /toaster_simu/add_entity "{id: 'IKEA_SHELF_DARK', name: 'IKEA_SHELF_DARK', type: 'object', ownerId: ''}"
  rosservice call /toaster_simu/set_entity_pose "{id: 'IKEA_SHELF_DARK', ownerId: '', type: 'object', pose:
  {position:
    {x: 5.5,
    y: 5.0,
    z: 0.0},
    orientation:
      {x: 0.0,
      y: 0.0,
      z: 0.7,
      w: 0.7}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE', name: 'RED_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE', ownerId: '', type: 'object', pose:
{position:
  {x: 4.3,
  y: 3.4,
  z: 0.82},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'RED_CUBE2', name: 'RED_CUBE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'RED_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 5.2,
  y: 5.0,
  z: 0.82},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_CUBE', name: 'GREEN_CUBE', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE', ownerId: '', type: 'object', pose:
{position:
  {x: 4.1,
  y: 3.4,
  z: 0.82},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_CUBE2', name: 'GREEN_CUBE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_CUBE2', ownerId: '', type: 'object', pose:
{position:
  {x: 5.3,
  y: 5.1,
  z: 0.82},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"


rosservice call /toaster_simu/add_entity "{id: 'PINK_TRASHBIN', name: 'PINK_TRASHBIN', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PINK_TRASHBIN', ownerId: '', type: 'object', pose:
{position:
  {x: 4.2,
  y: 3.1,
  z: 0.82},
  orientation:
    {x: 0.0,
    y: 0.0,
    z: 0.7,
    w: 0.7}}}"


rosservice call /toaster_simu/add_entity "{id: 'GREEN_TRASHBIN', name: 'GREEN_TRASHBIN', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_TRASHBIN', ownerId: '', type: 'object', pose:
{position:
  {x: 5.7,
  y: 5.0,
  z: 0.82},
  orientation:
    {x: 0.0,
    y: 0.0,
    z: 0.7,
    w: 0.7}}}"


rosservice call /toaster_simu/add_entity "{id: 'HERAKLES_HUMAN1', name: 'HERAKLES_HUMAN1', type: 'human', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'HERAKLES_HUMAN1', ownerId: '', type: 'human', pose:
{position:
  {x: 6.6,
  y: 6.1,
  z: 0.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'rightHand', name: 'rightHand', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'rightHand', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 6.6,
  y: 6.45,
  z: 0.8},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'base', name: 'base', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'base', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 6.6,
  y: 6.1,
  z: 0.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'head', name: 'head', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'head', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 6.6,
  y: 6.1,
  z: 1.5},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /agent_monitor/monitor_all_agents "monitorAll: true"

rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'base', agentId: 'HERAKLES_HUMAN1'}"
rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'head', agentId: 'HERAKLES_HUMAN1'}"
rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'rightHand', agentId: 'HERAKLES_HUMAN1'}"

rosservice call /agent_monitor/add_joint_to_agent "{jointName: 'laser_tilt_mount_link', agentId: 'pr2'}"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'interaction'
  myOwner: 'pr2'
  areaType: ''
  factType: 'interaction'
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -2, y: 2, z: 0}
    - {x: -2, y: -2, z: 0}
    - {x: 2, y: -2, z: 0}
    - {x: 2, y: 2, z: 0}
    - {x: -2, y: 2, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 1
  name: 'scanArea'
  myOwner: 'TABLE_4'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -0.36, y: -0.7, z: 0}
    - {x: 0.36, y: -0.7, z: 0}
    - {x: 0.36, y: 0.7, z: 0}
    - {x: -0.36, y: 0.7, z: 0}
    - {x: -0.36, y: -0.7, z: 0}
  insideEntities: [0]"

rosservice call /database_manager/set_info "{add: true, infoType: 'FACT', agentId: 'PR2_ROBOT', facts: [
    {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'RED_CUBE2', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_CUBE2', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'PINK_TRASHBIN', targetId: 'red', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'color', propertyType: 'state', subProperty: '', subjectId: 'GREEN_TRASHBIN', targetId: 'green', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
    , {property: 'canScan', propertyType: 'state', subProperty: '', subjectId: 'PR2_ROBOT', targetId: 'true', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
], event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}, id: '', name: '', ownerId: ''}"

