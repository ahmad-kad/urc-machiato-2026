import { useState, useMemo } from 'react';
import StateNode from './StateNode';
import TransitionArrow from './TransitionArrow';
import {
  SystemState,
  AutonomousSubstate,
  EquipmentServicingSubstate,
  CalibrationSubstate,
  STATE_METADATA,
  SUBSTATE_METADATA,
  EQUIPMENT_SERVICING_METADATA,
  CALIBRATION_METADATA,
  getStateColor,
  getStatePath
} from '../../config/stateDefinitions';

/**
 * StateTree Component - Hierarchical state machine visualization
 *
 * Displays the URC 2026 state machine as an interactive tree with:
 * - Color-coded state nodes
 * - Animated transitions
 * - Hierarchical substate display
 * - Current state highlighting
 * - Transition validation
 */
const StateTree = ({
  currentState = SystemState.BOOT,
  currentSubstate = AutonomousSubstate.NONE,
  currentSubSubstate = EquipmentServicingSubstate.NONE,
  currentCalibrationSubstate = CalibrationSubstate.NONE,
  onStateTransition = null,
  className = ""
}) => {
  const [hoveredState, setHoveredState] = useState(null);
  const [selectedState, setSelectedState] = useState(null);

  // Calculate positions for state nodes in a tree layout
  const statePositions = useMemo(() => {
    const positions = {};

    // Main states positioned in a vertical column
    const mainStates = Object.values(SystemState);
    const startY = 50;
    const spacingY = 80;

    mainStates.forEach((state, index) => {
      positions[state] = {
        x: 150,
        y: startY + (index * spacingY),
        level: 0
      };
    });

    // Autonomous substates positioned to the right
    const autonomousSubstates = Object.values(AutonomousSubstate).filter(s => s !== 'NONE');
    const subStartY = positions[SystemState.AUTONOMOUS].y - 20;

    autonomousSubstates.forEach((substate, index) => {
      positions[substate] = {
        x: 350,
        y: subStartY + (index * 60),
        level: 1,
        parent: SystemState.AUTONOMOUS
      };
    });

    // Equipment servicing sub-substates
    const servicingSubstates = Object.values(EquipmentServicingSubstate).filter(s => s !== 'NONE');
    const equipmentState = positions[AutonomousSubstate.EQUIPMENT_SERVICING];

    servicingSubstates.forEach((substate, index) => {
      positions[substate] = {
        x: 550,
        y: equipmentState.y - 60 + (index * 40),
        level: 2,
        parent: AutonomousSubstate.EQUIPMENT_SERVICING
      };
    });

    // Calibration substates positioned to the right of CALIBRATION
    const calibrationSubstates = Object.values(CalibrationSubstate).filter(s => s !== 'NONE');
    const calibStartY = positions[SystemState.CALIBRATION].y - 40;

    calibrationSubstates.forEach((substate, index) => {
      positions[substate] = {
        x: 350,
        y: calibStartY + (index * 30),
        level: 1,
        parent: SystemState.CALIBRATION
      };
    });

    return positions;
  }, []);

  // Generate transition arrows between states
  const transitionArrows = useMemo(() => {
    const arrows = [];

    // Main state transitions
    Object.values(STATE_METADATA).forEach(metadata => {
      const fromPos = statePositions[metadata.state];

      metadata.allowedTransitions.forEach(toState => {
        const toPos = statePositions[toState];
        if (fromPos && toPos) {
          arrows.push({
            id: `${metadata.state}-${toState}`,
            from: metadata.state,
            to: toState,
            fromPos,
            toPos,
            isActive: currentState === metadata.state
          });
        }
      });
    });

    // Substate transitions (simplified - just connect to parent)
    Object.values(SUBSTATE_METADATA).forEach(subMeta => {
      if (subMeta !== SUBSTATE_METADATA.NONE) {
        const subPos = statePositions[subMeta];
        const parentPos = statePositions[SystemState.AUTONOMOUS];
        if (subPos && parentPos) {
          arrows.push({
            id: `autonomous-${subMeta}`,
            from: SystemState.AUTONOMOUS,
            to: subMeta,
            fromPos: parentPos,
            toPos: subPos,
            isActive: currentState === SystemState.AUTONOMOUS && currentSubstate === subMeta,
            isSubstate: true
          });
        }
      }
    });

    // Calibration substate transitions
    Object.values(CALIBRATION_METADATA).forEach(calMeta => {
      if (calMeta !== CALIBRATION_METADATA.NONE) {
        const calPos = statePositions[calMeta];
        const parentPos = statePositions[SystemState.CALIBRATION];
        if (calPos && parentPos) {
          arrows.push({
            id: `calibration-${calMeta}`,
            from: SystemState.CALIBRATION,
            to: calMeta,
            fromPos: parentPos,
            toPos: calPos,
            isActive: currentState === SystemState.CALIBRATION && currentCalibrationSubstate === calMeta,
            isSubstate: true
          });
        }
      }
    });

    return arrows;
  }, [currentState, currentSubstate, currentCalibrationSubstate, statePositions]);

  // Handle state node click
  const handleStateClick = (stateName) => {
    setSelectedState(stateName);

    // If it's a main state and we have an onStateTransition callback
    if (Object.values(SystemState).includes(stateName) && onStateTransition) {
      // Check if transition is valid
      const isValid = STATE_METADATA[currentState]?.allowedTransitions.includes(stateName);
      onStateTransition(stateName, isValid);
    }
  };

  // Get current state path for display
  const currentStatePath = getStatePath(currentState, currentSubstate, currentSubSubstate, currentCalibrationSubstate);

  return (
    <div className={`relative w-full h-full bg-muted/20 rounded-lg overflow-hidden ${className}`}>
      {/* Header with current state info */}
      <div className="absolute top-0 left-0 right-0 z-10 bg-card/90 backdrop-blur-sm border-b border-border p-3">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="font-semibold text-sm text-primary">Current State</h3>
            <p className="text-xs text-muted-foreground font-mono">{currentStatePath}</p>
          </div>
          <div className="text-right">
            <div className={`inline-block w-3 h-3 rounded-full ${getStateColor(currentState)}`} />
            <p className="text-xs text-muted-foreground mt-1">
              {STATE_METADATA[currentState]?.displayName || currentState}
            </p>
          </div>
        </div>
      </div>

      {/* SVG Canvas for state tree */}
      <svg
        className="absolute inset-0 w-full h-full"
        style={{ paddingTop: '80px' }}
        viewBox="0 0 700 500"
        preserveAspectRatio="xMidYMid meet"
      >
        {/* Transition arrows */}
        {transitionArrows.map(arrow => (
          <TransitionArrow
            key={arrow.id}
            fromPos={arrow.fromPos}
            toPos={arrow.toPos}
            isActive={arrow.isActive}
            isSubstate={arrow.isSubstate}
          />
        ))}

        {/* State nodes */}
        {Object.entries(statePositions).map(([stateName, position]) => {
          const isCurrent = (
            stateName === currentState ||
            stateName === currentSubstate ||
            stateName === currentSubSubstate ||
            stateName === currentCalibrationSubstate
          );

          const isHovered = hoveredState === stateName;
          const isSelected = selectedState === stateName;

          // Get metadata based on state type
          let metadata = STATE_METADATA[stateName];
          if (!metadata) metadata = SUBSTATE_METADATA[stateName];
          if (!metadata) metadata = EQUIPMENT_SERVICING_METADATA[stateName];
          if (!metadata) metadata = CALIBRATION_METADATA[stateName];

          return (
            <StateNode
              key={stateName}
              stateName={stateName}
              metadata={metadata}
              position={position}
              isCurrent={isCurrent}
              isHovered={isHovered}
              isSelected={isSelected}
              onClick={() => handleStateClick(stateName)}
              onHover={setHoveredState}
            />
          );
        })}
      </svg>

      {/* Legend */}
      <div className="absolute bottom-0 left-0 right-0 bg-card/90 backdrop-blur-sm border-t border-border p-3">
        <div className="flex flex-wrap gap-4 text-xs">
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 rounded-full state-boot" />
            <span>Boot</span>
          </div>
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 rounded-full state-calibration" />
            <span>Calibration</span>
          </div>
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 rounded-full state-idle" />
            <span>Idle</span>
          </div>
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 rounded-full state-teleoperation" />
            <span>Teleop</span>
          </div>
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 rounded-full state-autonomous" />
            <span>Autonomous</span>
          </div>
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 rounded-full state-safety" />
            <span>Safety</span>
          </div>
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 rounded-full state-shutdown" />
            <span>Shutdown</span>
          </div>
        </div>
      </div>

      {/* Hover tooltip */}
      {hoveredState && (
        <div className="absolute top-20 left-4 bg-card border border-border rounded-lg p-3 shadow-lg z-20 max-w-xs">
          <h4 className="font-semibold text-sm text-primary">{hoveredState}</h4>
          <p className="text-xs text-muted-foreground mt-1">
            {STATE_METADATA[hoveredState]?.description ||
             SUBSTATE_METADATA[hoveredState]?.description ||
             EQUIPMENT_SERVICING_METADATA[hoveredState]?.description ||
             CALIBRATION_METADATA[hoveredState]?.description}
          </p>
        </div>
      )}
    </div>
  );
};

export default StateTree;
