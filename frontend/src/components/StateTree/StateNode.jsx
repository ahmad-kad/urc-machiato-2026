import { useState } from 'react';
import { getStateColor } from '../../config/stateDefinitions';

/**
 * StateNode Component - Individual state node in the state tree
 *
 * Renders a state as a colored circle with text, supporting:
 * - Hover effects
 * - Selection highlighting
 * - Current state indication
 * - Click handling
 */
const StateNode = ({
  stateName,
  metadata,
  position,
  isCurrent = false,
  isHovered = false,
  isSelected = false,
  onClick,
  onHover
}) => {
  const [isNodeHovered, setIsNodeHovered] = useState(false);

  if (!metadata) {
    console.warn(`No metadata found for state: ${stateName}`);
    return null;
  }

  const { x, y, level } = position;
  const nodeSize = level === 0 ? 40 : level === 1 ? 30 : 25;
  const fontSize = level === 0 ? 10 : level === 1 ? 9 : 8;

  // Determine node appearance
  const baseColor = getStateColor(metadata.color || 'gray');
  const isActive = isCurrent || isHovered || isSelected || isNodeHovered;

  // Node styling
  const nodeClasses = `
    ${baseColor}
    ${isActive ? 'ring-4 ring-primary ring-opacity-50' : ''}
    ${isCurrent ? 'ring-2 ring-white ring-opacity-100 scale-110' : ''}
    cursor-pointer transition-all duration-200 ease-in-out
    hover:scale-105 hover:shadow-lg
  `.trim();

  // Text styling
  const textClasses = `
    fill-current text-white font-medium text-center
    ${isCurrent ? 'font-bold' : ''}
    select-none pointer-events-none
  `.trim();

  const handleMouseEnter = () => {
    setIsNodeHovered(true);
    onHover && onHover(stateName);
  };

  const handleMouseLeave = () => {
    setIsNodeHovered(false);
    onHover && onHover(null);
  };

  const handleClick = () => {
    onClick && onClick(stateName);
  };

  return (
    <g
      className="state-node"
      onMouseEnter={handleMouseEnter}
      onMouseLeave={handleMouseLeave}
      onClick={handleClick}
    >
      {/* Node circle */}
      <circle
        cx={x}
        cy={y}
        r={nodeSize / 2}
        className={nodeClasses}
      />

      {/* Node border for better visibility */}
      <circle
        cx={x}
        cy={y}
        r={nodeSize / 2}
        fill="none"
        stroke="rgba(255,255,255,0.3)"
        strokeWidth="1"
      />

      {/* State name text */}
      <text
        x={x}
        y={y}
        textAnchor="middle"
        dominantBaseline="middle"
        className={textClasses}
        style={{ fontSize: `${fontSize}px` }}
      >
        {metadata.displayName || stateName}
      </text>

      {/* Current state indicator */}
      {isCurrent && (
        <circle
          cx={x}
          cy={y}
          r={(nodeSize / 2) + 8}
          fill="none"
          stroke="#ffffff"
          strokeWidth="3"
          strokeDasharray="5,5"
          className="animate-pulse"
          opacity="0.8"
        />
      )}

      {/* Substate indicator for states with substates */}
      {metadata.hasSubstates && (
        <rect
          x={x + nodeSize / 2 + 2}
          y={y - 4}
          width="8"
          height="8"
          fill="#ffffff"
          rx="2"
          opacity="0.8"
        />
      )}

      {/* Terminal state indicator */}
      {metadata.isTerminal && (
        <polygon
          points={`${x + nodeSize / 2 + 2},${y - 6} ${x + nodeSize / 2 + 8},${y} ${x + nodeSize / 2 + 2},${y + 6}`}
          fill="#ff6b6b"
          opacity="0.8"
        />
      )}
    </g>
  );
};

export default StateNode;
