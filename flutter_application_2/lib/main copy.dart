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
import 'dart:io';
import 'package:yaml/yaml.dart';
import 'package:yaml_writer/yaml_writer.dart';
import 'package:flutter/material.dart';

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

  double systime = 0.0;
  double runtime = 0.0;
  double x = 0.0, y = 0.0, z = 0.0;
  double phi = 0.0, theta = 0.0, psi = 0.0;
  double u = 0.0, v = 0.0, w = 0.0;
  double p = 0.0, q = 0.0, r = 0.0;
  double? _baseTime;

  final List<double> timeStamps = [];
  final List<double> rollSeries = [];
  final List<double> pitchSeries = [];
  final List<double> yawSeries = [];
  final List<double> uSeries = [];
  final List<double> vSeries = [];
  final List<double> rSeries = [];

  bool isConnected = false;
  String connectionStatus = "Disconnected";
  String testStatus = "대기";

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
        "topic": "/navigation_data",
        "type": "mk3_msgs/msg/NavigationType"
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

      // final x = decoded['x']?.toDouble() ?? 0.0;
      // final y = decoded['y']?.toDouble() ?? 0.0;
      // odomPoints.add(Offset(x, -y));

      // final header = decoded['msg']['header']['stamp'];
      // final sec = header['sec'] ?? 0;
      // final nsec = header['nanosec'] ?? 0;
      // final time = sec + nsec / 1e9;
      // _baseTime ??= time;
      // final relTime = time - _baseTime!;

      // final test_time = decoded['msg']['time'];
      // timeStamps.add(test_time);

      // // velocity
      // u = decoded['u']?.toDouble() ?? 0.0;
      // v = decoded['v']?.toDouble() ?? 0.0;
      // final r = decoded['r']?.toDouble() ?? 0.0;

      // rollSeries.add(roll);
      // pitchSeries.add(pitch);
      // yawSeries.add(yaw);
      // uSeries.add(u);
      // vSeries.add(v);
      // rSeries.add(r); // 추가 필요 시 선언해야 함

      // if (timeStamps.length > 1000) {
      //   timeStamps.removeAt(0);
      //   rollSeries.removeAt(0);
      //   pitchSeries.removeAt(0);
      //   yawSeries.removeAt(0);
      //   uSeries.removeAt(0);
      //   vSeries.removeAt(0);
      //   rSeries.removeAt(0);
      // }

      systime = decoded['systime']?.toDouble() ?? 0.0;
      runtime = decoded['time']?.toDouble() ?? 0.0;

      x = decoded['x']?.toDouble() ?? 0.0;
      y = decoded['y']?.toDouble() ?? 0.0;
      z = decoded['z']?.toDouble() ?? 0.0;

      phi = decoded['phi']?.toDouble() ?? 0.0;
      theta = decoded['theta']?.toDouble() ?? 0.0;
      psi = decoded['psi']?.toDouble() ?? 0.0;

      u = decoded['u']?.toDouble() ?? 0.0;
      v = decoded['v']?.toDouble() ?? 0.0;
      w = decoded['w']?.toDouble() ?? 0.0;
      p = decoded['p']?.toDouble() ?? 0.0;
      q = decoded['q']?.toDouble() ?? 0.0;
      r = decoded['r']?.toDouble() ?? 0.0;

      if (odomPoints.length > 500) odomPoints.removeAt(0);
      notifyListeners();
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
class ConfigRow extends StatefulWidget {
  final String name;
  final String initialValue;
  final String description;
  final ValueChanged<String>? onChanged;

  const ConfigRow({
    Key? key,
    required this.name,
    required this.initialValue,
    required this.description,
    this.onChanged,
  }) : super(key: key);

  @override
  _ConfigRowState createState() => _ConfigRowState();
}

class _ConfigRowState extends State<ConfigRow> {
  late final TextEditingController _controller;

  @override
  void initState() {
    super.initState();
    _controller = TextEditingController(text: widget.initialValue);

    // 커서가 맨 뒤로 이동하도록 설정 (입력 끊김 방지)
    _controller.selection = TextSelection.fromPosition(
      TextPosition(offset: _controller.text.length),
    );
  }

  @override
  void didUpdateWidget(covariant ConfigRow oldWidget) {
    super.didUpdateWidget(oldWidget);
    // 초기값이 바뀐 경우만 텍스트 변경
    if (oldWidget.initialValue != widget.initialValue) {
      _controller.text = widget.initialValue;
      _controller.selection = TextSelection.fromPosition(
        TextPosition(offset: _controller.text.length),
      );
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6.0),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          Expanded(flex: 2, child: Text(widget.name)),
          Expanded(
            flex: 3,
            child: Tooltip(
              message: widget.description,
              waitDuration: const Duration(milliseconds: 300),
              child: TextField(
                controller: _controller,
                onChanged: widget.onChanged,
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
  // 파일 경로
  final String _slamConfigPath =
      '/home/siwon/ros2_ws/src/lidarslam_ros2/lidarslam/param/lidarslam.yaml';
  final String _lidarConfigPath =
      '/home/siwon/ros2_ws/src/ouster-ros/ouster-ros/config/driver_params.yaml';

  // 현재 config
  Map<String, dynamic> _scanMatcherConfig = {};
  Map<String, dynamic> _graphSlamConfig = {};
  Map<String, dynamic> _lidarConfig = {
    'sensor_hostname': '',
    'udp_dest': '',
  };

  Future<void> _loadSlamConfigFromFile() async {
    try {
      final slamFile = File(_slamConfigPath);
      final slamContent = await slamFile.readAsString();
      final slamData = loadYaml(slamContent);

      final lidarFile = File(_lidarConfigPath);
      final lidarContent = await lidarFile.readAsString();
      final lidarData = loadYaml(lidarContent);

      final allParams = Map<String, dynamic>.from(
        lidarData['ouster/os_driver']['ros__parameters'] ?? {},
      );

      setState(() {
        _scanMatcherConfig = Map<String, dynamic>.from(
          slamData['scan_matcher']['ros__parameters'] ?? {},
        );
        _graphSlamConfig = Map<String, dynamic>.from(
          slamData['graph_based_slam']['ros__parameters'] ?? {},
        );
        _lidarConfig = {
          'sensor_hostname': allParams['sensor_hostname'] ?? '',
          'udp_dest': allParams['udp_dest'] ?? '',
        };
      });
    } catch (e) {
      _snack('Failed to load config: $e');
    }
  }

  Future<void> _saveSlamConfigToFile() async {
    try {
      final writer = YAMLWriter();

      // SLAM 설정 저장 ------------------------------
      final slamWrapped = {
        'scan_matcher': {
          'ros__parameters': _scanMatcherConfig,
        },
        'graph_based_slam': {
          'ros__parameters': _graphSlamConfig,
        },
      };
      final slamYaml = writer.write(slamWrapped);
      final slamFile = File(_slamConfigPath);
      await slamFile.writeAsString(slamYaml);

      // LiDAR 설정 병합 저장 -----------------------
      final lidarFile = File(_lidarConfigPath);
      final existing = await lidarFile.readAsString();
      final parsed = loadYaml(existing);

      // 기존 전체를 Map으로 변환
      final Map<String, dynamic> merged =
          Map<String, dynamic>.from(parsed ?? {});
      final Map<String, dynamic> osDriver =
          Map<String, dynamic>.from(merged['ouster/os_driver'] ?? {});

      final Map<String, dynamic> rosParams =
          Map<String, dynamic>.from(osDriver['ros__parameters'] ?? {});

      // 원하는 key만 덮어쓰기
      rosParams['sensor_hostname'] = _lidarConfig['sensor_hostname'];
      rosParams['udp_dest'] = _lidarConfig['udp_dest'];

      // 다시 병합
      osDriver['ros__parameters'] = rosParams;
      merged['ouster/os_driver'] = osDriver;

      final lidarYaml = writer.write(merged);
      await lidarFile.writeAsString(lidarYaml);

      _snack('Configuration saved!');
    } catch (e) {
      _snack('Save failed: $e');
    }
  }

  void _snack(String msg) {
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  @override
  void initState() {
    super.initState();
    _loadSlamConfigFromFile();
  }

  void _saveConfig() {
    _saveSlamConfigToFile();
  }

  dynamic _parseDynamic(String val) {
    if (val.toLowerCase() == 'true') return true;
    if (val.toLowerCase() == 'false') return false;
    final numVal = num.tryParse(val);
    return numVal ?? val;
  }

  late final Map<String, String> _desc = parameterDescriptions;

  bool showLidarConfig = false;
  bool showSlamConfig = false;

  Future<void> _launchSlam() async {
    try {
      final res =
          await http.post(Uri.parse('http://localhost:5001/launch_lidarslam'));
      _snack(res.statusCode == 200 ? '운동 계측 시작' : 'Failed – ${res.body}');
    } catch (_) {
      _snack('Connection error');
    }
  }

  Future<void> _stopSlam() async {
    try {
      final res =
          await http.post(Uri.parse('http://localhost:5001/stop_lidarslam'));
      _snack(res.statusCode == 200 ? '운동 계측 종료' : 'Stop failed – ${res.body}');
    } catch (_) {
      _snack('Connection error');
    }
  }

  Future<void> _launchLiDAR() async {
    try {
      final res =
          await http.post(Uri.parse('http://localhost:5001/launch_lidar'));
      _snack(res.statusCode == 200 ? 'LiDAR 센서 활성화' : '센서 실행 실패 – ${res.body}');
    } catch (_) {
      _snack('Connection error');
    }
  }

  Future<void> _stopLiDAR() async {
    try {
      final res =
          await http.post(Uri.parse('http://localhost:5001/stop_lidar'));
      _snack(
          res.statusCode == 200 ? 'LiDAR 센서 비활성화' : '센서 종료 실패 – ${res.body}');
    } catch (_) {
      _snack('Connection error');
    }
  }

  // UI ----------------------------------------------------------------------
  @override
  Widget build(BuildContext context) {
    final appState = context.watch<MyAppState>();
    return Stack(
      children: [
        AnimatedPositioned(
          duration: const Duration(milliseconds: 200),
          left: (showLidarConfig || showSlamConfig) ? -150 : 0,
          right: (showLidarConfig || showSlamConfig) ? 300 : 0,
          top: 0,
          bottom: 0,
          child: Center(
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    // 상태 & 연결 버튼 ------------------------------------------
                    Container(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 16, vertical: 12),
                      decoration: BoxDecoration(
                        color: Colors.white,
                        borderRadius: BorderRadius.circular(12),
                        boxShadow: [
                          BoxShadow(
                            color: Colors.black12,
                            blurRadius: 8,
                            offset: Offset(0, 2),
                          ),
                        ],
                      ),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Text(
                            "ROS 상태: ${appState.connectionStatus}",
                            style: const TextStyle(fontWeight: FontWeight.bold),
                          ),
                          const SizedBox(width: 16),
                          ElevatedButton(
                            onPressed: () async {
                              if (!appState.isConnected) {
                                final res = await http.post(Uri.parse(
                                    'http://localhost:5001/launch_rosbridge'));
                                if (res.statusCode == 200 ||
                                    res.statusCode == 400) {
                                  await Future.delayed(Duration(seconds: 3));
                                  appState.connectToROSBridge();
                                  _snack('ROS Bridge launched');
                                }
                              } else {
                                final res = await http.post(Uri.parse(
                                    'http://localhost:5001/stop_rosbridge'));
                                if (res.statusCode == 200) {
                                  appState.isConnected = false;
                                  appState.connectionStatus = "Disconnected";
                                  appState.notifyListeners();
                                  _snack('ROS Bridge stopped');
                                }
                              }
                            },
                            child: Text(
                                appState.isConnected ? "ROS 정지" : "시작 & 연결"),
                          ),
                        ],
                      ),
                    ),
                    const SizedBox(height: 24),

                    // 설정 버튼 --------------------------------------------------
                    Row(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        ElevatedButton.icon(
                          onPressed: () {
                            setState(() {
                              showLidarConfig = !showLidarConfig;
                              showSlamConfig = false;
                            });
                          },
                          icon: const Icon(Icons.settings),
                          label: const Text("LiDAR 세팅"),
                        ),
                        const SizedBox(width: 16),
                        ElevatedButton.icon(
                          onPressed: () {
                            setState(() {
                              showSlamConfig = !showSlamConfig;
                              showLidarConfig = false;
                            });
                          },
                          icon: const Icon(Icons.settings),
                          label: const Text("SLAM 세팅"),
                        ),
                      ],
                    ),
                    const SizedBox(height: 24),

                    // 라이다 실행/중지 버튼 -------------------------------------
                    Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        ElevatedButton.icon(
                          icon: const Icon(Icons.sensors_outlined),
                          label: const Text('LiDAR 활성화'),
                          onPressed: _launchLiDAR,
                        ),
                        const SizedBox(width: 16),
                        ElevatedButton.icon(
                          icon:
                              const Icon(Icons.stop_circle, color: Colors.red),
                          label: const Text('LiDAR 비활성화'),
                          onPressed: _stopLiDAR,
                        ),
                      ],
                    ),
                    const SizedBox(height: 24),
                    // const BigCard(),
                    // const SizedBox(height: 24),
                    Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        ElevatedButton.icon(
                          icon: const Icon(Icons.map),
                          label: const Text('운동 계측 시작'),
                          onPressed: _launchSlam,
                        ),
                        const SizedBox(width: 16),
                        ElevatedButton.icon(
                          icon:
                              const Icon(Icons.stop_circle, color: Colors.red),
                          label: const Text('운동 계측 종료'),
                          onPressed: _stopSlam,
                        ),
                      ],
                    ),
                  ],
                ),
                const SizedBox(width: 24),
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Container(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 16, vertical: 12),
                      decoration: BoxDecoration(
                        color: Colors.white,
                        borderRadius: BorderRadius.circular(12),
                        boxShadow: [
                          BoxShadow(
                            color: Colors.black12,
                            blurRadius: 8,
                            offset: Offset(0, 2),
                          ),
                        ],
                      ),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Text(
                            "시험 상태: ${appState.testStatus}",
                            style: const TextStyle(fontWeight: FontWeight.bold),
                          ),
                          const SizedBox(width: 16),
                          ElevatedButton(
                            onPressed: () async {},
                            child: Text(appState.isConnected
                                ? "시험 타이머 종료"
                                : "시험 타이머 시작"),
                          ),
                        ],
                      ),
                    ),
                    const SizedBox(height: 24),
                    Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        ElevatedButton.icon(
                          icon: const Icon(Icons.send),
                          label: const Text('데이터 수신'),
                          onPressed: _launchSlam,
                        ),
                        const SizedBox(width: 16),
                        ElevatedButton.icon(
                          icon:
                              const Icon(Icons.stop_circle, color: Colors.red),
                          label: const Text('수신 정지'),
                          onPressed: _stopSlam,
                        ),
                      ],
                    ),
                    const SizedBox(height: 24),
                    Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        ElevatedButton.icon(
                          icon: const Icon(Icons.send_and_archive),
                          label: const Text('데이터 저장'),
                          onPressed: _launchSlam,
                        ),
                        const SizedBox(width: 16),
                        ElevatedButton.icon(
                          icon:
                              const Icon(Icons.stop_circle, color: Colors.red),
                          label: const Text('저장 정지'),
                          onPressed: _stopSlam,
                        ),
                      ],
                    ),
                  ],
                )
              ],
            ),
          ),
        ),

        // 설정창 슬라이드 ---------------------------------------------------
        if (showLidarConfig || showSlamConfig)
          Positioned(
            right: 0,
            top: 0,
            bottom: 0,
            width: 500,
            child: AnimatedSwitcher(
              duration: const Duration(milliseconds: 100),
              transitionBuilder: (child, animation) {
                return SlideTransition(
                  position: Tween<Offset>(
                    begin: const Offset(1, 0),
                    end: Offset.zero,
                  ).animate(animation),
                  child: child,
                );
              },
              child: Container(
                key: ValueKey(showLidarConfig ? 'lidar' : 'slam'),
                color: Colors.white,
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Configuration',
                        style: Theme.of(context).textTheme.headlineSmall),
                    const SizedBox(height: 12),
                    Expanded(
                      child: ListView(
                        children: [
                          if (showLidarConfig)
                            ..._lidarConfig.entries.map((e) => ConfigRow(
                                  name: e.key,
                                  initialValue: '${e.value}',
                                  description: _desc[e.key] ?? '-',
                                  onChanged: (val) {
                                    setState(() {
                                      _lidarConfig[e.key] = val;
                                    });
                                  },
                                )),
                          if (showSlamConfig) ...[
                            const SizedBox(height: 8),
                            const Text(
                              'Scan Matcher Parameters',
                              style: TextStyle(
                                  fontWeight: FontWeight.bold, fontSize: 16),
                            ),
                            const Divider(),
                            ..._scanMatcherConfig.entries.map((e) => ConfigRow(
                                  name: e.key,
                                  initialValue: '${e.value}',
                                  description: _desc[e.key] ?? '-',
                                  onChanged: (val) {
                                    setState(() {
                                      _scanMatcherConfig[e.key] =
                                          _parseDynamic(val);
                                    });
                                  },
                                )),
                            const SizedBox(height: 16),
                            const Text(
                              'Graph-Based SLAM Parameters',
                              style: TextStyle(
                                  fontWeight: FontWeight.bold, fontSize: 16),
                            ),
                            const Divider(),
                            ..._graphSlamConfig.entries.map((e) => ConfigRow(
                                  name: e.key,
                                  initialValue: '${e.value}',
                                  description: _desc[e.key] ?? '-',
                                  onChanged: (val) {
                                    setState(() {
                                      _graphSlamConfig[e.key] =
                                          _parseDynamic(val);
                                    });
                                  },
                                )),
                          ],
                        ],
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
          ),
        Column(
          mainAxisAlignment: MainAxisAlignment.spaceBetween, // ⬅️ 이걸로 위/아래 배치
          children: [
            // ... 기존 연결/버튼 UI ...
            _buildDataPanel(appState), // 하단 고정
          ],
        ),
      ],
    );
  }

  Widget _buildDataPanel(MyAppState state) {
    String format(double v) => v.toStringAsFixed(3);

    return Card(
      margin: const EdgeInsets.only(top: 32),
      elevation: 3,
      color: Colors.white,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          children: [
            Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              const Text("📋 시험 상태",
                  style: TextStyle(fontWeight: FontWeight.bold)),
              const SizedBox(height: 8),
              Text("Sys Time [s]: ${format(state.systime)}"),
              Text("Run Time [s]: ${format(state.runtime)}"),
            ]),
            const SizedBox(width: 24),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text("📈 계측 데이터",
                    style: TextStyle(fontWeight: FontWeight.bold)),
                const SizedBox(height: 8),
                Text(
                    "x: ${format(state.x)}, y: ${format(state.y)}, z: ${format(state.z)}"),
                Text(
                    "phi: ${format(state.phi)}, theta: ${format(state.theta)}, yaw: ${format(state.psi)}"),
              ],
            ),
            const SizedBox(width: 24),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text("🧮 계산 데이터",
                    style: TextStyle(fontWeight: FontWeight.bold)),
                const SizedBox(height: 8),
                Text(
                    "u: ${format(state.u)}, v: ${format(state.v)}, w: ${format(state.w)}"),
                Text(
                    "p: ${format(state.p)}, q: ${format(state.q)}, r: ${format(state.r)}"),
              ],
            ),
          ],
        ),
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
                    _buildChart("r", appState.timeStamps, appState.rSeries),
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
