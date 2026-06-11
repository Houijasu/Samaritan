namespace Samaritan.Simulation.Rendering;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

/// <summary>
/// Renders a reference grid for the simulation.
/// The world origin is centered in the viewport, matching EntityRenderer.
/// </summary>
public class GridRenderer
{
    private readonly GraphicsDevice _device;
    private readonly BasicEffect _effect;
    private readonly float _scale;

    private VertexPositionColor[]? _gridVertices;
    private int _gridLineCount;
    private int _builtViewportWidth;
    private int _builtViewportHeight;

    public GridRenderer(GraphicsDevice device, float scale)
    {
        _device = device;
        _scale = scale;

        _effect = new BasicEffect(device)
        {
            VertexColorEnabled = true,
            View = Matrix.Identity,
            World = Matrix.Identity
        };

        BuildGrid();
    }

    private void BuildGrid()
    {
        var viewport = _device.Viewport;
        _effect.Projection = Matrix.CreateOrthographicOffCenter(0, viewport.Width, viewport.Height, 0, 0, 1);

        // World origin sits at the center of the viewport
        var offsetX = viewport.Width / 2f;
        var offsetY = viewport.Height / 2f;

        // Grid settings
        const int cellSize = 100; // World units
        const int gridExtent = 2000; // World units from center

        var lines = new List<VertexPositionColor>();
        var gridColor = new Color(40, 40, 50);
        var axisColor = new Color(60, 60, 80);

        // Vertical lines
        for (var x = -gridExtent; x <= gridExtent; x += cellSize)
        {
            var color = x == 0 ? axisColor : gridColor;
            var screenX = x * _scale + offsetX;
            var screenTop = -gridExtent * _scale + offsetY;
            var screenBottom = gridExtent * _scale + offsetY;

            lines.Add(new VertexPositionColor(new Vector3(screenX, screenTop, 0), color));
            lines.Add(new VertexPositionColor(new Vector3(screenX, screenBottom, 0), color));
        }

        // Horizontal lines
        for (var y = -gridExtent; y <= gridExtent; y += cellSize)
        {
            var color = y == 0 ? axisColor : gridColor;
            var screenY = y * _scale + offsetY;
            var screenLeft = -gridExtent * _scale + offsetX;
            var screenRight = gridExtent * _scale + offsetX;

            lines.Add(new VertexPositionColor(new Vector3(screenLeft, screenY, 0), color));
            lines.Add(new VertexPositionColor(new Vector3(screenRight, screenY, 0), color));
        }

        _gridVertices = lines.ToArray();
        _gridLineCount = _gridVertices.Length / 2;
        _builtViewportWidth = viewport.Width;
        _builtViewportHeight = viewport.Height;
    }

    public void Draw()
    {
        // Rebuild if the viewport changed (e.g. window resize)
        var viewport = _device.Viewport;
        if (viewport.Width != _builtViewportWidth || viewport.Height != _builtViewportHeight)
        {
            BuildGrid();
        }

        if (_gridVertices is null || _gridLineCount == 0) return;

        foreach (var pass in _effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            _device.DrawUserPrimitives(PrimitiveType.LineList, _gridVertices, 0, _gridLineCount);
        }
    }
}
