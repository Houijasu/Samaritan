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
    private Point2D _editCasterPosition;
    private Point2D _editTargetStart;
    private Vector2D _editTargetVelocity;
    private const double EditTargetSpeed = 350.0; // Default movement speed

    // World to screen transform (1 unit = 0.5 pixels, centered)
    private const float WorldScale = 0.5f;
    private Vector2 _worldOffset;

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

        // Create renderers
        _gridRenderer = new GridRenderer(GraphicsDevice, WorldScale, _worldOffset);
        _entityRenderer = new EntityRenderer(GraphicsDevice, WorldScale, _worldOffset);
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

        // Direction handle position (200 units in velocity direction from target start)
        var directionHandlePos = _editTargetVelocity.Length > 1
            ? _editTargetStart + _editTargetVelocity.Normalize().ScaleBy(200)
            : new Point2D(_editTargetStart.X + 200, _editTargetStart.Y);

        const double grabRadius = 80.0; // In world units (80 * 0.5 = 40 pixels)

        if (mousePressed && !wasPressed)
        {
            // Mouse just pressed - check what to grab
            var distToCaster = mouseWorld.DistanceTo(_editCasterPosition);
            var distToTargetStart = mouseWorld.DistanceTo(_editTargetStart);
            var distToCurrentTarget = mouseWorld.DistanceTo(_runner.State.TargetPosition);
            var distToDirection = mouseWorld.DistanceTo(directionHandlePos);

            if (distToCaster < grabRadius)
            {
                _dragMode = DragMode.Caster;
                _isPaused = true;
            }
            else if (distToTargetStart < grabRadius || distToCurrentTarget < grabRadius)
            {
                _dragMode = DragMode.Target;
                _isPaused = true;
                // Snap start to mouse immediately and update
                _editTargetStart = mouseWorld;
                var editedScenario = CreateEditedScenario();
                _runner.LoadScenario(editedScenario);
            }
            else if (distToDirection < grabRadius)
            {
                _dragMode = DragMode.Direction;
                _isPaused = true;
            }
        }
        else if (!mousePressed && wasPressed)
        {
            // Mouse released - apply changes and reload scenario
            if (_dragMode != DragMode.None)
            {
                var editedScenario = CreateEditedScenario();
                _runner.LoadScenario(editedScenario);
                _dragMode = DragMode.None;
            }
        }
        else if (mousePressed && _dragMode != DragMode.None)
        {
            // Dragging - update positions
            bool changed = false;
            switch (_dragMode)
            {
                case DragMode.Caster:
                    _editCasterPosition = mouseWorld;
                    changed = true;
                    break;
                case DragMode.Target:
                    _editTargetStart = mouseWorld;
                    changed = true;
                    break;
                case DragMode.Direction:
                    // Direction is relative to target start, scaled to speed
                    var direction = (mouseWorld - _editTargetStart);
                    if (direction.Length > 10)
                    {
                        _editTargetVelocity = direction.Normalize().ScaleBy(EditTargetSpeed);
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

        // Determine which scenario to draw (base or edited)
        var scenarioToDraw = _dragMode != DragMode.None
            ? CreateEditedScenario()
            : _scenarios[_currentScenarioIndex];

        // Draw simulation entities
        _entityRenderer.Draw(_runner.State, scenarioToDraw);

        // Always draw drag handles so user knows they can interact
        var directionHandlePos = _editTargetVelocity.Length > 1
            ? _editTargetStart + _editTargetVelocity.Normalize().ScaleBy(200)
            : new Point2D(_editTargetStart.X + 200, _editTargetStart.Y);

        _entityRenderer.DrawDragHandles(
            _editCasterPosition,
            _editTargetStart,
            directionHandlePos,
            _dragMode);

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
