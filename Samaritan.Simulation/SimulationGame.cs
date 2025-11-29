namespace Samaritan.Simulation;

using MathNet.Spatial.Euclidean;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

using Samaritan.Simulation.Core;
using Samaritan.Simulation.Rendering;
using Samaritan.Simulation.Scenarios;

/// <summary>
/// What the user is currently dragging.
/// </summary>
public enum DragMode
{
    None,
    Caster,
    Target,
    Direction
}

/// <summary>
/// Main MonoGame entry point for the skillshot prediction simulation.
/// </summary>
public class SimulationGame : Game
{
    private readonly GraphicsDeviceManager _graphics;
    private SpriteBatch _spriteBatch = null!;

    private GridRenderer _gridRenderer = null!;
    private EntityRenderer _entityRenderer = null!;
    private HudRenderer _hudRenderer = null!;

    private SimulationRunner _runner = null!;
    private Scenario[] _scenarios = null!;
    private int _currentScenarioIndex;

    private KeyboardState _previousKeyState;
    private MouseState _previousMouseState;
    private double _simulationSpeed = 0.5; // Start slower for observation
    private bool _isPaused;

    // Interactive editing
    private DragMode _dragMode = DragMode.None;
    private DragMode _hoverMode = DragMode.None;
    private Point2D _editCasterPosition;
    private Point2D _editTargetStart;
    private Vector2D _editTargetVelocity;
    private Vector2D _editDirection = new(1, 0); // Remember direction even when stationary
    private Vector2D _dragOffset; // Offset from handle center to mouse when drag started
    private const double EditTargetSpeed = 350.0; // Default movement speed

    // World to screen transform (1 unit = 0.5 pixels, centered)
    private const float WorldScale = 0.5f;

    public SimulationGame()
    {
        _graphics = new GraphicsDeviceManager(this)
        {
            PreferredBackBufferWidth = 1280,
            PreferredBackBufferHeight = 720,
            SynchronizeWithVerticalRetrace = true
        };

        Content.RootDirectory = "Content";
        IsMouseVisible = true;
        IsFixedTimeStep = true;
        TargetElapsedTime = TimeSpan.FromSeconds(1.0 / 60.0);
    }

    protected override void Initialize()
    {
        Window.Title = "Samaritan - Skillshot Prediction Simulation";

        // Center the world origin on screen
        // _worldOffset is now calculated dynamically in ScreenToWorld based on current Viewport
        // _worldOffset = new Vector2(
        //    _graphics.PreferredBackBufferWidth / 2f,
        //    _graphics.PreferredBackBufferHeight / 2f);

        base.Initialize();
    }

    protected override void LoadContent()
    {
        _spriteBatch = new SpriteBatch(GraphicsDevice);

        // Create renderers (offset is calculated dynamically based on viewport)
        _gridRenderer = new GridRenderer(GraphicsDevice, WorldScale, Vector2.Zero);
        _entityRenderer = new EntityRenderer(GraphicsDevice, WorldScale, Vector2.Zero);
        _hudRenderer = new HudRenderer(GraphicsDevice, _spriteBatch);

        // Load scenarios - start with scenario 2 (moving target) for better demo
        _scenarios = BuiltInScenarios.GetAll();
        _currentScenarioIndex = 1; // Start with "Linear vs Walking (Perpendicular)"

        // Initialize simulation runner
        _runner = new SimulationRunner();
        LoadCurrentScenario();

        // Initialize previous mouse state
        _previousMouseState = Mouse.GetState();
    }

    private void LoadCurrentScenario()
    {
        var scenario = _scenarios[_currentScenarioIndex];

        // Initialize edit positions from scenario
        _editCasterPosition = scenario.CasterPosition;
        _editTargetStart = scenario.TargetMovement.GetPosition(0);
        _editTargetVelocity = scenario.TargetMovement.GetVelocity(0);

        // Remember direction for the direction handle
        if (_editTargetVelocity.Length > 1)
        {
            _editDirection = _editTargetVelocity.Normalize();
        }
        else
        {
            _editDirection = new Vector2D(1, 0); // Default right
        }

        _runner.LoadScenario(scenario);
        _isPaused = false; // Auto-start so user can see target moving
    }

    private Scenario CreateEditedScenario()
    {
        var baseScenario = _scenarios[_currentScenarioIndex];

        // Create a linear movement pattern with current edit values
        var velocity = _editTargetVelocity.Length > 1 ? _editTargetVelocity : new Vector2D(0, 0);
        var movement = velocity.Length > 1
            ? new MovementPattern.Linear(_editTargetStart, velocity, 5.0)
            : (MovementPattern)new MovementPattern.Stationary(_editTargetStart);

        return baseScenario with
        {
            CasterPosition = _editCasterPosition,
            TargetMovement = movement
        };
    }

    protected override void Update(GameTime gameTime)
    {
        var keyState = Keyboard.GetState();
        var mouseState = Mouse.GetState();

        // Exit
        if (keyState.IsKeyDown(Keys.Escape))
        {
            Exit();
        }

        // Scenario switching
        if (WasKeyPressed(keyState, Keys.Left))
        {
            _currentScenarioIndex = (_currentScenarioIndex - 1 + _scenarios.Length) % _scenarios.Length;
            LoadCurrentScenario();
        }
        if (WasKeyPressed(keyState, Keys.Right))
        {
            _currentScenarioIndex = (_currentScenarioIndex + 1) % _scenarios.Length;
            LoadCurrentScenario();
        }

        // Mouse interaction for dragging
        HandleMouseInput(mouseState);

        // Update title with debug info
        // Window.Title = $"Samaritan - Mode: {_dragMode} Hover: {_hoverMode}";

        // Playback controls
        if (WasKeyPressed(keyState, Keys.Space))
        {
            if (_runner.State.Phase == SimulationPhase.Complete)
            {
                _runner.Reset();
            }
            _isPaused = !_isPaused;
        }

        if (WasKeyPressed(keyState, Keys.R))
        {
            _runner.Reset();
            _isPaused = true;
        }

        if (WasKeyPressed(keyState, Keys.P))
        {
            _isPaused = !_isPaused;
        }

        // Speed controls
        if (WasKeyPressed(keyState, Keys.OemPlus) || WasKeyPressed(keyState, Keys.Add))
        {
            _simulationSpeed = Math.Min(4.0, _simulationSpeed * 2);
        }
        if (WasKeyPressed(keyState, Keys.OemMinus) || WasKeyPressed(keyState, Keys.Subtract))
        {
            _simulationSpeed = Math.Max(0.25, _simulationSpeed / 2);
        }

        // Update simulation
        if (!_isPaused)
        {
            var dt = gameTime.ElapsedGameTime.TotalSeconds * _simulationSpeed;
            _runner.Update(dt);
        }

        _previousKeyState = keyState;
        _previousMouseState = mouseState;
        base.Update(gameTime);
    }

    private void HandleMouseInput(MouseState mouseState)
    {
        var mouseWorld = ScreenToWorld(mouseState.X, mouseState.Y);
        var mousePressed = mouseState.LeftButton == ButtonState.Pressed;
        var wasPressed = _previousMouseState.LeftButton == ButtonState.Pressed;

        // Direction handle position (200 units in direction from target start)
        var directionHandlePos = _editTargetStart + _editDirection.ScaleBy(200);

        const double grabRadius = 80.0; // In world units (80 * 0.5 = 40 pixels)

        // Update hover mode if not dragging
        if (_dragMode == DragMode.None)
        {
            _hoverMode = DragMode.None;
            var minDist = double.MaxValue;

            var distToCaster = mouseWorld.DistanceTo(_editCasterPosition);
            var distToTarget = mouseWorld.DistanceTo(_editTargetStart);
            var distToDirection = mouseWorld.DistanceTo(directionHandlePos);

            if (distToCaster < grabRadius && distToCaster < minDist)
            {
                minDist = distToCaster;
                _hoverMode = DragMode.Caster;
            }
            if (distToTarget < grabRadius && distToTarget < minDist)
            {
                minDist = distToTarget;
                _hoverMode = DragMode.Target;
            }
            if (distToDirection < grabRadius && distToDirection < minDist)
            {
                minDist = distToDirection;
                _hoverMode = DragMode.Direction;
            }
        }

        if (mousePressed && !wasPressed)
        {
            // Mouse just pressed - try to grab hovered item
            if (_hoverMode != DragMode.None)
            {
                _dragMode = _hoverMode;
                _isPaused = true;

                Point2D handlePos = default;
                switch (_dragMode)
                {
                    case DragMode.Caster: handlePos = _editCasterPosition; break;
                    case DragMode.Target: handlePos = _editTargetStart; break;
                    case DragMode.Direction: handlePos = directionHandlePos; break;
                }

                // Store offset from handle center to mouse position
                _dragOffset = mouseWorld - handlePos;
            }
        }
        else if (!mousePressed && wasPressed)
        {
            // Mouse released - finalize drag
            _dragMode = DragMode.None;
        }
        else if (mousePressed && _dragMode != DragMode.None)
        {
            // Dragging - update positions with offset
            var targetPos = mouseWorld - _dragOffset;
            bool changed = false;

            switch (_dragMode)
            {
                case DragMode.Caster:
                    _editCasterPosition = targetPos;
                    changed = true;
                    break;
                case DragMode.Target:
                    _editTargetStart = targetPos;
                    changed = true;
                    break;
                case DragMode.Direction:
                    // Direction is from target start to handle position
                    var direction = targetPos - _editTargetStart;
                    if (direction.Length > 10)
                    {
                        _editDirection = direction.Normalize();
                        _editTargetVelocity = _editDirection.ScaleBy(EditTargetSpeed);
                    }
                    else
                    {
                        _editTargetVelocity = new Vector2D(0, 0);
                    }
                    changed = true;
                    break;
            }

            if (changed)
            {
                var editedScenario = CreateEditedScenario();
                _runner.LoadScenario(editedScenario);
            }
        }
    }

    private Point2D ScreenToWorld(int screenX, int screenY)
    {
        var viewport = GraphicsDevice.Viewport;
        var offsetX = viewport.Width / 2f;
        var offsetY = viewport.Height / 2f;

        var worldX = (screenX - offsetX) / WorldScale;
        var worldY = (screenY - offsetY) / WorldScale;
        return new Point2D(worldX, worldY);
    }

    private bool WasKeyPressed(KeyboardState current, Keys key)
    {
        return current.IsKeyDown(key) && _previousKeyState.IsKeyUp(key);
    }

    protected override void Draw(GameTime gameTime)
    {
        GraphicsDevice.Clear(new Color(20, 20, 25));

        // Draw grid
        _gridRenderer.Draw();

        // Always use edited scenario so entities match the drag handles
        var scenarioToDraw = CreateEditedScenario();

        // Draw simulation entities
        _entityRenderer.Draw(_runner.State, scenarioToDraw);

        // Direction handle position (200 units in direction from target start)
        var directionHandlePos = _editTargetStart + _editDirection.ScaleBy(200);

        _entityRenderer.DrawDragHandles(
            _editCasterPosition,
            _editTargetStart,
            directionHandlePos,
            _dragMode != DragMode.None ? _dragMode : _hoverMode);

        // Draw HUD
        _hudRenderer.Draw(
            scenarioToDraw,
            _runner.State,
            _currentScenarioIndex,
            _scenarios.Length,
            _simulationSpeed,
            _isPaused);

        base.Draw(gameTime);
    }
}
