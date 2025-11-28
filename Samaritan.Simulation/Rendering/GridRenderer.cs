namespace Samaritan.Simulation.Rendering;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

/// <summary>
/// Renders a reference grid for the simulation.
/// </summary>
public class GridRenderer
{
    private readonly GraphicsDevice _device;
    private readonly BasicEffect _effect;
    private readonly float _scale;
    private readonly Vector2 _offset;

    private VertexPositionColor[]? _gridVertices;
    private int _gridLineCount;

    public GridRenderer(GraphicsDevice device, float scale, Vector2 offset)
    {
        _device = device;
        _scale = scale;
        _offset = offset;

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
            var screenX = x * _scale + _offset.X;
            var screenTop = -gridExtent * _scale + _offset.Y;
            var screenBottom = gridExtent * _scale + _offset.Y;

            lines.Add(new VertexPositionColor(new Vector3(screenX, screenTop, 0), color));
            lines.Add(new VertexPositionColor(new Vector3(screenX, screenBottom, 0), color));
        }

        // Horizontal lines
        for (var y = -gridExtent; y <= gridExtent; y += cellSize)
        {
            var color = y == 0 ? axisColor : gridColor;
            var screenY = y * _scale + _offset.Y;
            var screenLeft = -gridExtent * _scale + _offset.X;
            var screenRight = gridExtent * _scale + _offset.X;

            lines.Add(new VertexPositionColor(new Vector3(screenLeft, screenY, 0), color));
            lines.Add(new VertexPositionColor(new Vector3(screenRight, screenY, 0), color));
        }

        _gridVertices = lines.ToArray();
        _gridLineCount = _gridVertices.Length / 2;
    }

    public void Draw()
    {
        if (_gridVertices is null || _gridLineCount == 0) return;

        foreach (var pass in _effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            _device.DrawUserPrimitives(PrimitiveType.LineList, _gridVertices, 0, _gridLineCount);
        }
    }
}
