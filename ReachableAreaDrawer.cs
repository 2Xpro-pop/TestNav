using Godot;
using System;
using System.Collections.Generic;
using System.Linq;

public partial class ReachableAreaDrawer : Node2D
{
    // === Настройки ===
    [Export] public Color FillColor { get; set; } = new Color(0.15f, 0.75f, 0.45f, 0.35f);
    [Export] public Color StrokeColor { get; set; } = new Color(0.05f, 0.5f, 0.3f, 0.9f);
    [Export] public float StrokeWidth { get; set; } = 2f;

    // Твой CellSize (важно: var для float/double, как ты просишь)
    public const float CellSize = 55f;

    // Твоё «поле» 0/1. Здесь — как пример (то, что прислал):
    private static readonly string[] _rows =
    {
        "000000000000000000000",
        "000000011111110000000",
        "000011111111111110000",
        "000011111111111110000",
        "000111111111111111000",
        "000111111111111111000",
        "001111111111111111100",
        "001111111111111111100",
        "001111111111111111100",
        "001111111101111111100",
        "001111111100000111100",
        "001111111000111111100",
        "000111111111111111000",
        "000111111111001110000",
        "000011111111000000000",
        "000011111111000000000",
        "000000011000000000000",
    };

    // Сгенерированные данные рендера
    private List<Triangle> _triangles = new();
    private List<Contour> _contours = new();

    private struct Triangle
    {
        public Vector2 A, B, C;
    }

    private struct Contour
    {
        public Vector2[] Points; // замкнутый контур (последняя точка != первая)
        public bool IsHole;
    }

    public override void _Ready()
    {
        BuildGeometryFromGrid();
        QueueRedraw(); // перерисовать
    }

    public override void _Draw()
    {
        // === Заливка: рисуем треугольники ===
        foreach (var t in _triangles)
        {
            DrawPolygon(new Vector2[] { t.A, t.B, t.C }, new Color[] { FillColor, FillColor, FillColor });
        }

        // === Обводка: обходим все контуры (внешний и дырки) ===
        foreach (var c in _contours)
        {
            // polyline с замыканием
            for (var i = 0; i < c.Points.Length; i++)
            {
                var p0 = c.Points[i];
                var p1 = c.Points[(i + 1) % c.Points.Length];
                DrawLine(p0, p1, StrokeColor, StrokeWidth, antialiased: true);
            }
        }
    }

    private void BuildGeometryFromGrid()
    {
        var height = _rows.Length;
        var width = _rows[0].Length;

        // 1) Создаём BitMap из грида (1 = проходимо => непрозрачно)
        var bmp = new Bitmap();
        bmp.Create(new(width, height));

        for (var y = 0; y < height; y++)
        {
            var row = _rows[y];
            for (var x = 0; x < width; x++)
            {
                var isWalkable = row[x] == '1';
                bmp.SetBit(x, y, isWalkable);
            }
        }

        // 2) Извлекаем полигоны (внешние контуры и дырки).
        // В Godot 4 есть BitMap.OpaqueToPolygons(...).
        // Подпись может чуть отличаться в минорных версиях:
        //   OpaqueToPolygons(float epsilon = 2.0f, float threshold = 0.5f)
        // Возвращает массив полигонов (каждый — массив точек), где каждый полигон уже
        // может содержать "holes" (в некоторых версиях возвращаются отдельно).
        //
        // Универсальный путь: получить «контуры» через GetContours() для 1-зон и для 0-зон внутри.
        // Ниже — безопасный вариант, использующий OpaqueToPolygons, а затем дополнительно
        // выделяющий дырки через инверсию.
        var epsilon = 0.01f; // минимальное упрощение, чтобы сохранить форму клеток
        var outerPolys = bmp.OpaqueToPolygons(new Rect2I(0, 0, width, height), epsilon); // список полигонов "1"
        // Для дырок: инвертируем и берём "0-острова", затем фильтруем только те, что лежат внутри outer
        var inv = InvertBitMap(bmp);
        var zeroIslands = inv.OpaqueToPolygons(new Rect2I(0, 0, width, height), epsilon);

        // 3) Масштабируем точки в мировые координаты (CellSize), сдвигаем, если нужно
        var scaledOuter = outerPolys
            .Select(poly => poly.Select(p => ToWorld(p)).ToList())
            .ToList();

        var scaledIslands = zeroIslands
            .Select(poly => poly.Select(p => ToWorld(p)).ToList())
            .ToList();

        // 4) Для каждого outer-полигона найдём его «дыры» — те zero-острова, чьи точки лежат внутри outer.
        // Простой критерий: берём первую точку дырки и проверяем PointInPolygon для outer.
        var allTriList = new List<Triangle>();
        var allContours = new List<Contour>();

        foreach (var outer in scaledOuter)
        {
            var holesForOuter = new List<List<Vector2>>();
            foreach (var zIsland in scaledIslands)
            {
                // Центроид как "надёжная" внутренняя точка
                var centroid = zIsland.Aggregate(Vector2.Zero, (acc, v) => acc + v) / zIsland.Count;
                // небольшое смещение к центру чтобы уйти от границы
                var dir = (centroid - zIsland[0]);
                if (dir.LengthSquared() < 1e-6f) dir = new Vector2(0.001f, 0.001f);
                var testPoint = zIsland[0] + dir.Normalized() * (0.1f * CellSize);

                // Используем Geometry2D.IsPointInPolygon
                if (Geometry2D.IsPointInPolygon(testPoint, outer.ToArray()))
                    holesForOuter.Add(zIsland); ;
            }

            // === Сохраняем контуры: внешний + дырки ===
            allContours.Add(new Contour { Points = MakeClosed(outer), IsHole = false });
            foreach (var h in holesForOuter)
                allContours.Add(new Contour { Points = MakeClosed(h), IsHole = true });

            // === Триангуляция с дырками ===
            // В Godot 4 есть Geometry2D.TriangulatePolygon(outer, holes)
            // Возвращает список треугольников (как индексы в объединённом массиве),
            // но в некоторых билдах — сразу массив точек треугольников. Покроем оба случая.

            // Собираем объединённый массив вершин: outer + все holes подряд, плюс offsets
            // === Triangulation via boolean subtraction: outer − holes (with tiny inset on holes) ===
            var nav = new NavigationPolygon();

            // Внешний контур CCW, дырки CW
            var outerCCW = EnsureOrientation(outer, ccw: true);
            nav.AddOutline(outerCCW);

            foreach (var h in holesForOuter)
            {
                var holeCW = EnsureOrientation(h, ccw: false);
                nav.AddOutline(holeCW);
            }

            // Да, метод помечен [Obsolete], но он строит полигоны из контуров (с учётом дыр)
#pragma warning disable 0618
            nav.MakePolygonsFromOutlines();
#pragma warning restore 0618

            // Забираем вершины и проходим все выпуклые полигоны
            var verts = nav.GetVertices();              // PackedVector2Array
            var polyCount = nav.GetPolygonCount();      // int

            for (var p = 0; p < polyCount; p++)
            {
                var polyIdx = nav.GetPolygon(p);        // int[]
                if (polyIdx.Length < 3) continue;

                // Фан-триангуляция выпуклого полигона
                var a = verts[polyIdx[0]];
                for (var i = 1; i < polyIdx.Length - 1; i++)
                {
                    var b = verts[polyIdx[i]];
                    var c = verts[polyIdx[i + 1]];
                    allTriList.Add(new Triangle { A = a, B = b, C = c });
                }
            }
        }

        _triangles = allTriList;
        _contours = allContours;
    }

    private static Vector2[] EnsureOrientation(List<Vector2> path, bool ccw)
    {
        var arr = path.ToArray();
        var area = SignedArea(arr);
        var isCcw = area > 0f;
        if (isCcw != ccw) Array.Reverse(arr);
        return arr;
    }

    // Перегрузка SignedArea для массива (ты уже имеешь версию для List<Vector2>)
    private static float SignedArea(Vector2[] p)
    {
        var s = 0f;
        for (var i = 0; i < p.Length; i++)
        {
            var a = p[i];
            var b = p[(i + 1) % p.Length];
            s += (a.X * b.Y - b.X * a.Y);
        }
        return 0.5f * s; // >0 => CCW
    }

    // === ВСПОМОГАТЕЛЬНЫЕ ===

    private static Vector2 ToWorld(Vector2 p)
    {
        // масштабируем каждую «клеточную вершину» в мировые координаты.
        var x = p.X * CellSize;
        var y = p.Y * CellSize;
        var vx = (float)x;
        var vy = (float)y;
        return new Vector2(vx, vy);
    }

    private static Bitmap InvertBitMap(Bitmap src)
    {
        var size = src.GetSize();
        var dst = new Bitmap();
        dst.Create(new(size.X, size.Y));
        for (var y = 0; y < size.Y; y++)
            for (var x = 0; x < size.X; x++)
            {
                var v = src.GetBit(x, y);
                dst.SetBit(x, y, !v);
            }
        return dst;
    }

    private static bool PointInPolygon(Vector2 p, List<Vector2> poly)
    {
        // чётно-нечётный алгоритм
        var inside = false;
        var count = poly.Count;
        for (var i = 0; i < count; i++)
        {
            var a = poly[i];
            var b = poly[(i + 1) % count];
            var cond = (a.Y > p.Y) != (b.Y > p.Y);
            if (cond)
            {
                var t = (p.Y - a.Y) / (b.Y - a.Y);
                var xCross = a.X + t * (b.X - a.X);
                if (xCross > p.X) inside = !inside;
            }
        }
        return inside;
    }

    private static Vector2[] MakeClosed(List<Vector2> path)
    {
        // Убедимся, что последняя точка не дублирует первую
        if (path.Count > 1 && path[0].IsEqualApprox(path[^1]))
            return path.ToArray();
        var list = new List<Vector2>(path);
        list.Add(path[0]);
        return list.ToArray();
    }

    // Булева разность outer − holes (заглушка: в Godot 4 есть Geometry2D.ClipPolygons/Exclude/Intersect/Merge).
    // Здесь оставляем «крючок» — если метод доступен, он вернёт набор простых контуров.
    private static List<List<Vector2>> ClipSubtract(List<Vector2> outer, List<List<Vector2>> holes)
    {
        try
        {
            // Если твоя сборка содержит Geometry2D.ExcludePolygons:
            // var res = Geometry2D.ExcludePolygons(outer.ToArray(), holes.Select(h => h.ToArray()).ToArray());
            // return res.Select(r => r.ToList()).ToList();
        }
        catch { /* ниже фоллбек */ }

        // Мини-фоллбек: если нет булевых операций — просто вернём outer
        // (дырок не будет видно в заливке; это только как аварийный вариант).
        return new List<List<Vector2>> { outer };
    }

    // Простой Ear-Clipping для КОНВЕКСНЫХ/умеренно вогнутых простых контуров (без самопересечений).
    // Для нашей сетки обычно ок, но если фигуры совсем «зубчатые», лучше использовать родной триангулятор.
    private static List<Triangle> EarClip(List<Vector2> poly)
    {
        var tris = new List<Triangle>();
        var pts = new List<Vector2>(poly);

        // Убедимся, что полигон CCW
        if (SignedArea(pts) < 0) pts.Reverse();

        var guard = 0;
        while (pts.Count >= 3 && guard++ < 100000)
        {
            var earFound = false;
            for (var i = 0; i < pts.Count; i++)
            {
                var prev = pts[(i - 1 + pts.Count) % pts.Count];
                var curr = pts[i];
                var next = pts[(i + 1) % pts.Count];

                if (!IsConvex(prev, curr, next))
                    continue;

                var ear = true;
                for (var j = 0; j < pts.Count; j++)
                {
                    if (j == i) continue;
                    var p = pts[j];
                    if (PointInTri(p, prev, curr, next))
                    {
                        ear = false;
                        break;
                    }
                }

                if (!ear) continue;

                tris.Add(new Triangle { A = prev, B = curr, C = next });
                pts.RemoveAt(i);
                earFound = true;
                break;
            }

            if (!earFound) break; // деградация — выходим
        }

        return tris;
    }

    private static float SignedArea(List<Vector2> p)
    {
        var s = 0f;
        for (var i = 0; i < p.Count; i++)
        {
            var a = p[i];
            var b = p[(i + 1) % p.Count];
            s += (a.X * b.Y - b.X * a.Y);
        }
        var area = 0.5f * s;
        var v = area;
        return v;
    }

    private static bool IsConvex(Vector2 a, Vector2 b, Vector2 c)
    {
        var cross = (b - a).Cross(c - b);
        var v = cross;
        return v > 0f; // для CCW
    }

    private static bool PointInTri(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
    {
        var v0 = c - a;
        var v1 = b - a;
        var v2 = p - a;

        var den = v0.X * v1.Y - v1.X * v0.Y;
        if (Math.Abs(den) < 1e-7f) return false;

        var u = (v2.X * v1.Y - v1.X * v2.Y) / den;
        var v = (v0.X * v2.Y - v2.X * v0.Y) / den;

        var u2 = u;
        var v3 = v;
        return u2 >= 0f && v3 >= 0f && (u2 + v3) <= 1f + 1e-6f;
    }

    // Делает маленький "инсет" (усадку) полигона вовнутрь на delta (>0)
    // Возвращает первый контур результата (обычно один), либо исходный, если оффсет не удался.
    private static Vector2[] InsetHole(List<Vector2> hole, float delta)
    {
        // Geometry2D.OffsetPolygon принимает положительное значение для расширения наружу,
        // поэтому для "усадки" даём отрицательное.
        try
        {
            var res = Geometry2D.OffsetPolygon(hole.ToArray(), -delta);
            if (res != null && res.Count > 0 && res[0].Length >= 3)
                return res[0];
        }
        catch { /* бывает, что не удаётся из-за самопересечений */ }
        return hole.ToArray();
    }
}