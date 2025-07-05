import 'package:english_words/english_words.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'dart:convert';
import 'package:web_socket_channel/io.dart';
import 'dart:math';
import 'package:web_socket_channel/status.dart' as status;
import 'package:fl_chart/fl_chart.dart';
import 'package:http/http.dart' as http;
import 'parameter_descriptions.dart'; // 설명 Map

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (context) => MyAppState(),
      child: MaterialApp(
        title: 'Namer App',
        theme: ThemeData(
          useMaterial3: true,
          colorScheme: ColorScheme.fromSeed(
              seedColor: const Color.fromRGBO(0, 255, 0, 1.0)),
        ),
        home: MyHomePage(),
      ),
    );
  }
}

class MyAppState extends ChangeNotifier {
  var current = WordPair.random();
  var favorites = <WordPair>[];

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double u = 0.0;
  double v = 0.0;
  double? _baseTime;

  final List<double> timeStamps = [];
  final List<double> rollSeries = [];
  final List<double> pitchSeries = [];
  final List<double> yawSeries = [];
  final List<double> uSeries = [];
  final List<double> vSeries = [];

  bool isConnected = false;
  String connectionStatus = "Disconnected";

  IOWebSocketChannel? channel;

  void connectToROSBridge() async {
    try {
      connectionStatus = "Connecting...";
      notifyListeners();

      // WebSocket 연결 자체를 try-catch로 감싸야 함
      final newChannel = IOWebSocketChannel.connect('ws://localhost:9090');

      // subscribe 요청
      newChannel.sink.add(jsonEncode({
        "op": "subscribe",
        "topic": "/ekf/odometry",
        "type": "nav_msgs/msg/Odometry"
      }));

      // stream listen
      newChannel.stream.listen(
        (data) {
          _handleIncomingData(data);
        },
        onDone: () {
          connectionStatus = "Disconnected";
          isConnected = false;
          notifyListeners();
        },
        onError: (error) {
          print("WebSocket stream error: $error");
          connectionStatus = "Connection Failed";
          isConnected = false;
          notifyListeners();
        },
        cancelOnError: true,
      );

      // 정상 연결되면 저장
      channel = newChannel;
      isConnected = true;
      connectionStatus = "Connected";
      notifyListeners();
    } catch (e, st) {
      print("WebSocket connection exception: $e");
      print(st); // 🔍 예외 추적에 도움 됨
      connectionStatus = "Connection Failed";
      isConnected = false;
      notifyListeners();
    }
  }

  void _handleIncomingData(dynamic data) {
    try {
      final decoded = jsonDecode(data);

      final x =
          decoded['msg']['pose']['pose']['position']['x']?.toDouble() ?? 0.0;
      final y =
          decoded['msg']['pose']['pose']['position']['y']?.toDouble() ?? 0.0;
      odomPoints.add(Offset(x, -y));

      final header = decoded['msg']['header']['stamp'];
      final sec = header['sec'] ?? 0;
      final nsec = header['nanosec'] ?? 0;
      final time = sec + nsec / 1e9;
      _baseTime ??= time;
      final relTime = time - _baseTime!;
      timeStamps.add(relTime);

      rollSeries.add(roll);
      pitchSeries.add(pitch);
      yawSeries.add(yaw);
      uSeries.add(u);
      vSeries.add(v);

      // 버퍼 제한
      if (timeStamps.length > 1000) {
        timeStamps.removeAt(0);
        rollSeries.removeAt(0);
        pitchSeries.removeAt(0);
        yawSeries.removeAt(0);
        uSeries.removeAt(0);
        vSeries.removeAt(0);
      }

      final q = decoded['msg']['pose']['pose']['orientation'];
      final xq = q['x']?.toDouble() ?? 0.0;
      final yq = q['y']?.toDouble() ?? 0.0;
      final zq = q['z']?.toDouble() ?? 0.0;
      final wq = q['w']?.toDouble() ?? 1.0;

      u = decoded['msg']['twist']['twist']['linear']['x']?.toDouble() ?? 0.0;
      v = decoded['msg']['twist']['twist']['linear']['y']?.toDouble() ?? 0.0;

      final euler = eulerFromQuaternion(xq, yq, zq, wq);
      roll = euler[0] * 180 / pi;
      pitch = euler[1] * 180 / pi;
      yaw = euler[2] * 180 / pi;

      if (odomPoints.length > 500) odomPoints.removeAt(0);
      notifyListeners(); // 데이터 갱신 알림
    } catch (e) {
      print("Parse error: $e");
    }
  }

  // ↓ Add this.
  void getNext() {
    current = WordPair.random();
    notifyListeners(); // 보고 있는 사람에게 알림을 보내는 매서드
  }

  void toggleFavorite() {
    if (favorites.contains(current)) {
      favorites.remove(current);
    } else {
      favorites.add(current);
    }
    notifyListeners();
  }

  final List<Offset> odomPoints = [];
}

class MyHomePage extends StatefulWidget {
  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  var selectedIndex = 0;

  @override
  Widget build(BuildContext context) {
    Widget page;
    switch (selectedIndex) {
      case 0:
        page = SlamDashboard();
        break;
      case 1:
        page = Sensor();
        break;
      case 2:
        page = DataMoniter();
        break;
      default:
        throw UnimplementedError('no widget for $selectedIndex');
    }

    return LayoutBuilder(builder: (context, constraints) {
      return Scaffold(
        body: Row(
          children: [
            // 🟩 NavigationRail + Toggle Column
            Column(
              children: [
                Expanded(
                  child: NavigationRail(
                    // extended: constraints.maxWidth >= 1000,
                    extended: false,
                    destinations: const [
                      NavigationRailDestination(
                        icon: Icon(Icons.home),
                        label: Text('Home'),
                      ),
                      NavigationRailDestination(
                        icon: Icon(Icons.sensors_outlined),
                        label: Text('Sensor'),
                      ),
                      NavigationRailDestination(
                        icon: Icon(Icons.monitor),
                        label: Text('Data Moniter'),
                      ),
                    ],
                    selectedIndex: selectedIndex,
                    onDestinationSelected: (value) {
                      setState(() {
                        selectedIndex = value;
                      });
                    },
                  ),
                ),
              ],
            ),
            Expanded(
              // Expanded 위젯을 사용하면 일부 하위 요소는 필요한 만큼만 공간을 차지함.
              child: Container(
                color: Theme.of(context).colorScheme.primaryContainer,
                child: page,
              ),
            ),
          ],
        ),
      );
    });
  }
}

// ───────────────────────────────────────────────── BigCard ────────────────────
class BigCard extends StatelessWidget {
  const BigCard({super.key});

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Card(
      color: theme.colorScheme.primary,
      child: const Padding(
        padding: EdgeInsets.all(20.0),
        child: Text(
          'SLAM',
          style: TextStyle(fontSize: 48, color: Colors.white),
        ),
      ),
    );
  }
}

// ───────────────────────────────────────────────── ConfigRow ──────────────────
class ConfigRow extends StatelessWidget {
  const ConfigRow({
    super.key,
    required this.name,
    required this.initialValue,
    required this.description,
  });

  final String name;
  final String initialValue;
  final String description;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6.0),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          Expanded(flex: 2, child: Text(name)),
          Expanded(
            flex: 3,
            child: Tooltip(
              message: description,
              waitDuration: const Duration(milliseconds: 300),
              child: TextField(
                controller: TextEditingController(text: initialValue),
                decoration: const InputDecoration(
                  isDense: true,
                  border: OutlineInputBorder(),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

// ──────────────────────────────────────────────── SlamDashboard ───────────────
class SlamDashboard extends StatefulWidget {
  const SlamDashboard({super.key});

  @override
  State<SlamDashboard> createState() => _SlamDashboardState();
}

class _SlamDashboardState extends State<SlamDashboard> {
  // Parameters ---------------------------------------------------------------
  final Map<String, dynamic> _config = {
    'ndt_resolution': 2.0,
    'ndt_num_threads': 2,
    'gicp_corr_dist_threshold': 5.0,
    'trans_for_mapupdate': 1.5,
    'vg_size_for_input': 0.5,
    'vg_size_for_map': 0.1,
    'use_min_max_filter': true,
    'scan_min_range': 1.0,
    'scan_max_range': 200.0,
    'scan_period': 0.2,
    'set_initial_pose': true,
    'initial_pose_x': 0.0,
    'initial_pose_y': 0.0,
    'initial_pose_z': 0.0,
    'initial_pose_qx': 0.0,
    'initial_pose_qy': 0.0,
    'initial_pose_qz': 0.0,
    'initial_pose_qw': 1.0,
    'gb_ndt_resolution': 1.0,
    'gb_ndt_num_threads': 2,
    'voxel_leaf_size': 0.1,
    'loop_detection_period': 3000,
    'threshold_loop_closure_score': 0.7,
    'distance_loop_closure': 100.0,
    'range_of_searching_loop_closure': 20.0,
    'search_submap_num': 2,
    'num_adjacent_pose_constraints': 5,
    'use_save_map_in_loop': true,
    'debug_flag': true,
  };

  // Quick‑and‑dirty dummy descriptions --------------------------------------
  late final Map<String, String> _desc = parameterDescriptions;


  // Launch / stop actions ----------------------------------------------------
  Future<void> _launchSlam() async {
    try {
      final res = await http.post(Uri.parse('http://localhost:5001/launch_lidarslam'));
      _snack(res.statusCode == 200 ? 'LidarSLAM launched' : 'Failed – ${res.body}');
    } catch (_) {
      _snack('Connection error');
    }
  }

  Future<void> _stopSlam() async {
    try {
      final res = await http.post(Uri.parse('http://localhost:5001/stop_lidarslam'));
      _snack(res.statusCode == 200 ? 'LidarSLAM stopped' : 'Stop failed – ${res.body}');
    } catch (_) {
      _snack('Connection error');
    }
  }

  void _saveConfig() => _snack('Configuration saved!');
  void _snack(String msg) => ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));

  // UI ----------------------------------------------------------------------
  @override
  Widget build(BuildContext context) {
    return Center(
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          // Left pane -------------------------------------------------------
          Expanded(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const BigCard(),
                const SizedBox(height: 20),
                Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    ElevatedButton.icon(
                      onPressed: _launchSlam,
                      icon: const Icon(Icons.map),
                      label: const Text('Launch SLAM'),
                    ),
                    const SizedBox(width: 12),
                    ElevatedButton.icon(
                      style: ElevatedButton.styleFrom(backgroundColor: Colors.white),
                      onPressed: _stopSlam,
                      icon: const Icon(Icons.stop_circle, color: Colors.red),
                      label: const Text('Stop SLAM'),
                    ),
                  ],
                ),
              ],
            ),
          ),
          const VerticalDivider(thickness: 1, width: 1),
          // Right pane ------------------------------------------------------
          Expanded(
            child: Padding(
              padding: const EdgeInsets.all(24.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('Configuration', style: Theme.of(context).textTheme.headlineMedium),
                  const SizedBox(height: 12),
                  Expanded(
                    child: ListView(
                      children: _config.entries
                          .map((e) => ConfigRow(
                                name: e.key,
                                initialValue: '${e.value}',
                                description: _desc[e.key]!,
                              ))
                          .toList(),
                    ),
                  ),
                  const SizedBox(height: 12),
                  Align(
                    alignment: Alignment.centerRight,
                    child: ElevatedButton.icon(
                      onPressed: _saveConfig,
                      icon: const Icon(Icons.save),
                      label: const Text('Save'),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}


class Sensor extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    var appState = context.watch<MyAppState>();

    if (appState.favorites.isEmpty) {
      return Center(
        child: Text('No favorites yet.'),
      );
    }

    return ListView(
      children: [
        Padding(
          padding: const EdgeInsets.all(20),
          child: Text('You have '
              '${appState.favorites.length} favorites:'),
        ),
        for (var pair in appState.favorites)
          ListTile(
              leading: Icon(Icons.favorite), title: Text(pair.asLowerCase)),
      ],
    );
  }
}


class DataMoniter extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final appState = context.watch<MyAppState>();
    final points = appState.odomPoints;

    return Column(
      children: [
        // 🟢 상단 연결 버튼
        Padding(
          padding: const EdgeInsets.all(8.0),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                "ROS Status: ${appState.connectionStatus}",
                style: const TextStyle(fontWeight: FontWeight.bold),
              ),
              ElevatedButton(
                onPressed: () async {
                  if (!appState.isConnected) {
                    // ➕ 연결 시도
                    try {
                      final res = await http.post(
                        Uri.parse('http://localhost:5001/launch_rosbridge'),
                      );

                      if (res.statusCode == 200) {
                        print('✅ rosbridge launched!');
                        appState.connectToROSBridge(); // WebSocket 연결
                      } else {
                        print('Flask launch error: ${res.body}');
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                              content: Text('Failed to launch ROS bridge')),
                        );
                      }
                    } catch (e) {
                      print('HTTP exception: $e');
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                            content: Text('Connection to launcher failed')),
                      );
                    }
                  } else {
                    // 종료 시도
                    try {
                      final res = await http.post(
                        Uri.parse('http://localhost:5001/stop_rosbridge'),
                      );

                      if (res.statusCode == 200) {
                        print('rosbridge stopped!');
                        appState.isConnected = false;
                        appState.connectionStatus = "Disconnected";
                        appState.notifyListeners();
                      } else {
                        print('stop error: ${res.body}');
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(content: Text('Failed to stop ROS bridge')),
                        );
                      }
                    } catch (e) {
                      print('HTTP exception: $e');
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                            content: Text('Failed to contact stop server')),
                      );
                    }
                  }
                },
                child: Text(
                    appState.isConnected ? "Stop ROS" : "Launch & Connect"),
              ),
            ],
          ),
        ),

        // 🟦 본문: 그래프 Row (xy + 시계열)
        Expanded(
          child: Row(
            children: [
              // 좌측: 궤적 그래프
              Expanded(
                flex: 2,
                child: Center(
                  child: CustomPaint(
                    painter: PathPainter(points),
                    size: const Size(400, 400),
                  ),
                ),
              ),

              // 우측: 시계열 차트 모음
              Expanded(
                flex: 3,
                child: ListView(
                  padding: const EdgeInsets.all(8),
                  children: [
                    _buildChart(
                        "Roll", appState.timeStamps, appState.rollSeries),
                    _buildChart(
                        "Pitch", appState.timeStamps, appState.pitchSeries),
                    _buildChart("Yaw", appState.timeStamps, appState.yawSeries),
                    _buildChart("u", appState.timeStamps, appState.uSeries),
                    _buildChart("v", appState.timeStamps, appState.vSeries),
                    _buildChart("r", appState.timeStamps,
                        appState.vSeries), // 필요 시 rSeries로 수정
                  ],
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildChart(String label, List<double> times, List<double> values) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: const TextStyle(fontWeight: FontWeight.bold)),
        SizedBox(
          height: 150,
          child: LineChart(buildLineChart(times, values)),
        ),
        const Divider(),
      ],
    );
  }
}

class PathPainter extends CustomPainter {
  final List<Offset> points;
  PathPainter(this.points);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.blueAccent
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    final path = Path();
    if (points.isNotEmpty) {
      final Offset origin = Offset(size.width / 2, size.height / 2);
      path.moveTo(origin.dx + points[0].dx * 20, origin.dy + points[0].dy * 20);
      for (var p in points) {
        path.lineTo(origin.dx + p.dx * 20, origin.dy + p.dy * 20);
      }
    }
    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(PathPainter oldDelegate) => true;
}

List<double> eulerFromQuaternion(double x, double y, double z, double w) {
  final sinrCosp = 2.0 * (w * x + y * z);
  final cosrCosp = 1.0 - 2.0 * (x * x + y * y);
  final roll = atan2(sinrCosp, cosrCosp);

  final sinp = 2.0 * (w * y - z * x);
  final pitch = (sinp.abs() >= 1.0) ? (pi / 2.0) * sinp.sign : asin(sinp);

  final sinyCosp = 2.0 * (w * z + x * y);
  final cosyCosp = 1.0 - 2.0 * (y * y + z * z);
  final yaw = atan2(sinyCosp, cosyCosp);

  return [roll, pitch, yaw];
}

LineChartData buildLineChart(List<double> times, List<double> values) {
  return LineChartData(
    lineBarsData: [
      LineChartBarData(
        spots: List.generate(times.length, (i) => FlSpot(times[i], values[i])),
        isCurved: true,
        dotData: FlDotData(show: false),
        color: Color.fromRGBO(0, 0, 1, 1),
        barWidth: 2,
      ),
    ],
    titlesData: FlTitlesData(show: true),
    borderData: FlBorderData(show: true),
  );
}

Future<void> launchRosbridgeViaAPI() async {
  try {
    final res =
        await http.post(Uri.parse('http://localhost:5001/launch_rosbridge'));
    if (res.statusCode == 200) {
      print('rosbridge_server launched!');
    } else {
      print('Failed: ${res.body}');
    }
  } catch (e) {
    print('Error: $e');
  }
}
