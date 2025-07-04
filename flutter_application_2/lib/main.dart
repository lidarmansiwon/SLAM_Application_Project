import 'package:english_words/english_words.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'dart:convert';
import 'package:web_socket_channel/io.dart';
import 'dart:math';
import 'package:web_socket_channel/status.dart' as status;
import 'package:fl_chart/fl_chart.dart';
import 'package:http/http.dart' as http;

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
        page = GeneratorPage();
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

class GeneratorPage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    var appState = context.watch<MyAppState>();
    var pair = appState.current;

    IconData icon;
    if (appState.favorites.contains(pair)) {
      icon = Icons.favorite;
    } else {
      icon = Icons.favorite_border;
    }

    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          BigCard(pair: pair),
          SizedBox(height: 10),
          Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              ElevatedButton.icon(
                onPressed: () async {
                  try {
                    final res = await http.post(
                      Uri.parse('http://localhost:5001/launch_lidarslam'),
                    );
                    if (res.statusCode == 200) {
                      print('lidarslam launched!');
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('LidarSLAM launched')),
                      );
                    } else {
                      print('Failed to Launch: ${res.body}');
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('Failed to launch LidarSLAM')),
                      );
                    }
                  } catch (e) {
                    print('HTTP exception: $e');
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                          content: Text('Connection to ROS launcher failed')),
                    );
                  }
                },
                icon: const Icon(Icons.map),
                label: const Text('Launch SLAM'),
              ),
              SizedBox(width: 10),
              ElevatedButton.icon(
                onPressed: () async {
                  try {
                    final res = await http.post(
                        Uri.parse('http://localhost:5001/stop_lidarslam'));
                    if (res.statusCode == 200) {
                      print('✅ LidarSLAM stopped');
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('LidarSLAM stopped')),
                      );
                    } else {
                      print('Failed to stop: ${res.body}');
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('Stop failed: ${res.body}')),
                      );
                    }
                  } catch (e) {
                    print('Exception: $e');
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(content: Text('Connection error')),
                    );
                  }
                },
                icon: Icon(Icons.stop_circle, color: Colors.red),
                label: Text('Stop SLAM'),
              ),
            ],
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

class BigCard extends StatelessWidget {
  const BigCard({
    super.key,
    required this.pair,
  });

  final WordPair pair;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    final style = theme.textTheme.displayMedium!.copyWith(
      color: theme.colorScheme.onPrimary,
    );

    return Card(
      color: theme.colorScheme.primary,
      child: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Text(
          pair.asLowerCase,
          style: style,
          semanticsLabel: "${pair.first} ${pair.second}",
        ),
      ),
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
  final sinr_cosp = 2.0 * (w * x + y * z);
  final cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  final roll = atan2(sinr_cosp, cosr_cosp);

  final sinp = 2.0 * (w * y - z * x);
  final pitch = (sinp.abs() >= 1.0) ? (pi / 2.0) * sinp.sign : asin(sinp);

  final siny_cosp = 2.0 * (w * z + x * y);
  final cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  final yaw = atan2(siny_cosp, cosy_cosp);

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
