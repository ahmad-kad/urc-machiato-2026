import { useROS } from './hooks/useROS';
import { useStateMachine } from './hooks/useStateMachine';
import { getStatusText } from './utils/rosbridge';
import StateTree, { StateControls } from './components/StateTree';

function App() {
  const {
    isConnected,
    connectionStatus,
    reconnectAttempts,
    lastError,
    connect,
    disconnect,
    maxReconnectAttempts,
    ros
  } = useROS({
    url: 'ws://localhost:9090',
    reconnectInterval: 3000,
    maxReconnectAttempts: 10
  });

  // State machine hook
  const {
    currentState,
    currentSubstate,
    currentSubSubstate,
    currentCalibrationSubstate,
    isTransitioning,
    lastTransition,
    requestStateTransition
  } = useStateMachine(ros);

  return (
    <div className="app-container">
      {/* Header */}
      <header className="app-header">
        <div className="header-left">
          <h1 className="app-title">URC 2026 Testing Interface</h1>
          <div className="connection-status">
            <div
              className={`status-dot ${
                isConnected ? 'status-connected' : 'status-disconnected'
              }`}
            />
            <span>{getStatusText(connectionStatus)}</span>
          </div>
          {isTransitioning && (
            <div className="connection-status">
              <div className="status-dot status-transitioning" />
              <span>Transitioning...</span>
            </div>
          )}
        </div>

        <div className="header-left">
          <span className="text-muted">Reconnect attempts: {reconnectAttempts}/{maxReconnectAttempts}</span>
          <div style={{ display: 'flex', gap: '0.5rem' }}>
            <button
              onClick={connect}
              className="btn btn-primary"
              disabled={isConnected}
            >
              Connect
            </button>
            <button
              onClick={disconnect}
              className="btn btn-destructive"
              disabled={!isConnected}
            >
              Disconnect
            </button>
          </div>
        </div>

        {lastTransition && (
          <div style={{
            marginTop: '0.5rem',
            padding: '0.5rem',
            backgroundColor: '#374151',
            border: '1px solid #4b5563',
            borderRadius: '0.25rem',
            fontSize: '0.75rem'
          }}>
            <span style={{ fontWeight: '500' }}>Last Transition:</span>{' '}
            {lastTransition.fromState} → {lastTransition.toState}
            {lastTransition.reason && ` (${lastTransition.reason})`}
            <span className={`ml-2 ${lastTransition.success ? 'text-success' : 'text-error'}`}>
              {lastTransition.success ? '✓' : '✗'}
            </span>
          </div>
        )}

        {lastError && (
          <div style={{
            marginTop: '0.5rem',
            padding: '0.5rem',
            backgroundColor: '#451a1a',
            border: '1px solid #7f1d1d',
            borderRadius: '0.25rem',
            fontSize: '0.875rem',
            color: '#fca5a5'
          }}>
            Connection Error: {lastError.message || 'Unknown error'}
          </div>
        )}
      </header>

      {/* Main Content */}
      <main style={{ flex: 1, padding: '1.5rem' }}>
        <div style={{ maxWidth: '1280px', margin: '0 auto' }}>
          <div className="app-main">
            {/* State Tree Panel */}
            <div className="panel">
              <div className="panel-header">
                <h2>State Machine</h2>
              </div>
              <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem', height: '100%' }}>
                {/* State Tree Visualization */}
                <div style={{ flex: 1 }}>
                  <StateTree
                    currentState={currentState}
                    currentSubstate={currentSubstate}
                    currentSubSubstate={currentSubSubstate}
                    currentCalibrationSubstate={currentCalibrationSubstate}
                    onStateTransition={async (targetState, isValid) => {
                      if (!isValid) {
                        console.warn(`Invalid transition requested: ${currentState} -> ${targetState}`);
                        return;
                      }

                      try {
                        console.log(`Requesting transition: ${currentState} -> ${targetState}`);
                        await requestStateTransition(targetState, 'frontend_request');
                        console.log('Transition request sent successfully');
                      } catch (error) {
                        console.error('Transition request failed:', error);
                      }
                    }}
                    className="h-48"
                  />
                </div>

                {/* State Controls */}
                <StateControls
                  currentState={currentState}
                  isTransitioning={isTransitioning}
                  onStateTransition={async (targetState, isValid, reason = 'frontend_request', force = false) => {
                    try {
                      console.log(`Requesting ${force ? 'forced ' : ''}transition: ${currentState} -> ${targetState}`);
                      await requestStateTransition(targetState, reason, force);
                      console.log('Transition request sent successfully');
                    } catch (error) {
                      console.error('Transition request failed:', error);
                      throw error; // Re-throw for StateControls to handle
                    }
                  }}
                />
              </div>
            </div>

            {/* 3D Visualization Panel */}
            <div className="panel">
              <div className="panel-header">
                <h2>3D Visualization</h2>
              </div>
              <div className="scene-container">
                <div className="scene-placeholder">
                  3D Scene (Three.js)
                  <br />
                  <small>Map, navigation paths, robot pose</small>
                </div>
              </div>
            </div>

            {/* Video Feed Panel */}
            <div className="panel">
              <div className="panel-header">
                <h2>Camera Feed</h2>
              </div>
              <div className="video-container">
                <div className="video-placeholder">
                  Video Stream
                  <br />
                  <small>Adaptive quality, bandwidth monitoring</small>
                </div>
              </div>
            </div>
          </div>

          {/* Test Sequence Panel */}
          <div className="test-panel">
            <div className="panel-header" style={{ marginBottom: '1rem' }}>
              <h2>Automated Test Sequences</h2>
            </div>
            <div className="test-grid">
              <button className="test-button">
                <div className="test-button-title">Boot → Calibration → Idle</div>
                <div className="test-button-desc">System initialization workflow</div>
              </button>
              <button className="test-button">
                <div className="test-button-title">Safety Transitions</div>
                <div className="test-button-desc">Emergency stop and recovery</div>
              </button>
              <button className="test-button">
                <div className="test-button-title">Mission Workflows</div>
                <div className="test-button-desc">Complete mission scenarios</div>
              </button>
            </div>
          </div>
        </div>
      </main>

      {/* Footer */}
      <footer className="app-footer">
        <div>
          URC 2026 Autonomous Rover Testing Interface
        </div>
        <div className="footer-stats">
          <span>Bandwidth: <span className="text-success">0.0 KB/s</span></span>
          <span>FPS: <span className="text-success">60</span></span>
          <span>Latency: <span className="text-success">50ms</span></span>
        </div>
      </footer>
    </div>
  );
}

export default App;
