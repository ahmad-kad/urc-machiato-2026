/**
 * TransitionArrow Component - Animated arrow between state nodes
 *
 * Renders SVG arrows with animation effects for state transitions
 */
const TransitionArrow = ({
  fromPos,
  toPos,
  isActive = false,
  isSubstate = false
}) => {
  const { x: x1, y: y1 } = fromPos;
  const { x: x2, y: y2, level } = toPos;

  // Calculate arrow path
  const dx = x2 - x1;
  const dy = y2 - y1;
  const distance = Math.sqrt(dx * dx + dy * dy);

  // Adjust arrow start/end points to avoid overlapping with nodes
  const nodeRadius = level === 0 ? 20 : level === 1 ? 15 : 12.5;
  const substateRadius = level === 0 ? 20 : 15;

  const fromRadius = isSubstate ? substateRadius : nodeRadius;
  const toRadius = nodeRadius;

  const angle = Math.atan2(dy, dx);
  const startX = x1 + Math.cos(angle) * fromRadius;
  const startY = y1 + Math.sin(angle) * fromRadius;
  const endX = x2 - Math.cos(angle) * toRadius;
  const endY = y2 - Math.sin(angle) * toRadius;

  // Create curved path for better visual flow
  const midX = (startX + endX) / 2;
  const midY = (startY + endY) / 2;
  const curvature = Math.min(distance * 0.3, 50); // Adaptive curvature
  const controlX = midX + (isSubstate ? curvature * 0.5 : 0);
  const controlY = midY;

  const pathData = `M ${startX} ${startY} Q ${controlX} ${controlY} ${endX} ${endY}`;

  // Arrowhead calculation
  const arrowLength = 8;
  const arrowWidth = 6;
  const arrowAngle = Math.PI / 6; // 30 degrees

  const arrowX1 = endX - arrowLength * Math.cos(angle - arrowAngle);
  const arrowY1 = endY - arrowLength * Math.sin(angle - arrowAngle);
  const arrowX2 = endX - arrowLength * Math.cos(angle + arrowAngle);
  const arrowY2 = endY - arrowLength * Math.sin(angle + arrowAngle);

  const arrowheadData = `M ${endX} ${endY} L ${arrowX1} ${arrowY1} L ${arrowX2} ${arrowY2} Z`;

  return (
    <g className="transition-arrow">
      {/* Main arrow line */}
      <path
        d={pathData}
        fill="none"
        stroke={isActive ? "#3b82f6" : "#6b7280"}
        strokeWidth={isActive ? "3" : "2"}
        strokeOpacity={isActive ? "1" : "0.4"}
        className={`
          transition-all duration-300 ease-in-out
          ${isActive ? 'animate-pulse' : ''}
        `}
        markerEnd="url(#arrowhead)"
      />

      {/* Arrowhead marker definition */}
      <defs>
        <marker
          id="arrowhead"
          markerWidth={arrowWidth}
          markerHeight={arrowLength}
          refX={arrowLength}
          refY={arrowLength / 2}
          orient="auto"
          markerUnits="strokeWidth"
        >
          <polygon
            points={`0,0 ${arrowLength},${arrowLength / 2} 0,${arrowLength}`}
            fill={isActive ? "#3b82f6" : "#6b7280"}
            fillOpacity={isActive ? "1" : "0.4"}
          />
        </marker>
      </defs>

      {/* Flow animation for active transitions */}
      {isActive && (
        <circle r="4" fill="#3b82f6" opacity="0.8">
          <animateMotion
            dur="2s"
            repeatCount="indefinite"
            path={pathData}
          />
        </circle>
      )}

      {/* Substate connection indicator */}
      {isSubstate && (
        <circle
          cx={startX}
          cy={startY}
          r="3"
          fill="#10b981"
          opacity="0.7"
          className="animate-ping"
        />
      )}
    </g>
  );
};

export default TransitionArrow;
