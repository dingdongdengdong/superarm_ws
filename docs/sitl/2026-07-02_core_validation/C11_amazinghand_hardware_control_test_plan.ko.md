# C11 - AmazingHandControl upstream 테스트 실행

## 목적

AmazingHand를 컴퓨터에 연결한 뒤, 별도 adapter를 새로 만들지 않고 `Betatester777/AmazingHandControl` repo에 들어있는 테스트/실행 프로그램이 이 컴퓨터와 실제 손에서 잘 동작하는지 확인한다.

이 task의 끝은 **upstream repo의 프로그램을 설치하고, 제공된 테스트 또는 CLI가 정상 동작한다는 산출물을 남기는 것**이다. 내부 servo 구조나 코드 분석은 이해에 도움이 되지만 필수 구현 목표가 아니다.

## 1. 사람용

### 사람이 이해해야 하는 것

완료 후 학생은 아래 질문에 답할 수 있어야 한다.

```text
Q1. AmazingHandControl repo를 어떤 commit에서 테스트했는가?
Q2. Python 환경과 dependency 설치가 성공했는가?
Q3. hardware 없이 실행되는 test가 통과했는가?
Q4. 실제 AmazingHand를 연결한 상태에서 어떤 명령 또는 hardware test를 실행했는가?
Q5. 실행 결과를 보고 "이 컴퓨터에서 이 repo의 control program을 쓸 수 있다/없다"를 어떻게 판단했는가?
```

### 기준 자료

| 항목 | 값 |
|---|---|
| 참고 repo | `https://github.com/Betatester777/AmazingHandControl` |
| 확인한 commit 예시 | `2a59fd8fbf521bcdf547cc48cc0f55f4b74ee697` |
| 주요 실행 파일 | `amazing_hand_cmd.py`, `amazing_hand_gui.py` |
| hardware test 파일 | `tests/test_system_hardware.py`, `tests/test_cmd_hardware.py` |
| Linux 기본 port | `/dev/ttyACM0` |
| baudrate | `1000000` |

### 사람이 직접 하는 부분

```text
[ ] AmazingHandControl repo를 clone한다.
[ ] 가상환경을 만들고 dependency를 설치한다.
[ ] `pytest`로 hardware 없이 돌아가는 test를 먼저 실행한다.
[ ] AmazingHand와 USB serial adapter를 연결한다.
[ ] 실제 port 이름을 확인한다.
[ ] repo README의 CLI 또는 hardware test 명령을 실행한다.
[ ] 실행 명령, 출력 요약, pass/fail, 막힌 이유를 산출물에 기록한다.
```

### AI에게 도움받을 수 있는 부분

AI는 실험의 주체가 아니라 보조 도구다. 필요할 때 아래 정도를 부탁할 수 있다.

```text
[ ] README에서 설치/실행 명령을 찾아 정리한다.
[ ] pytest 실패 로그를 읽고 원인 후보를 요약한다.
[ ] port permission 문제 같은 환경 이슈 해결 순서를 제안한다.
[ ] 산출물 markdown 초안을 정리한다.
```

AI에게 로그 분석이나 산출물 정리를 맡길 때는 아래 보조 문서를 참고할 수 있다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C11_amazinghand_hardware_control_test_plan.ai.ko.md
```

### 진행 순서

1. repo를 clone하고 commit hash를 기록한다.
2. Python 환경을 만들고 dependency를 설치한다.
3. hardware 없는 test를 실행한다.
4. 실제 AmazingHand를 연결한다.
5. README에 있는 CLI 또는 hardware test를 실행한다.
6. 결과를 산출물에 남기고 C12로 넘어갈지 사람이 판단한다.

### 실행 예시

```bash
git clone https://github.com/Betatester777/AmazingHandControl /tmp/AmazingHandControl
cd /tmp/AmazingHandControl
git rev-parse HEAD
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-dev.txt
pytest -q
python amazing_hand_cmd.py --list
pytest tests/test_system_hardware.py tests/test_cmd_hardware.py --hardware --port /dev/ttyACM0
```

실제 port가 `/dev/ttyUSB0`이면 마지막 명령의 `--port` 값을 바꾼다.

### 중단 기준

```text
[ ] dependency 설치가 실패한다.
[ ] hardware 없는 test가 대량 실패한다.
[ ] serial port가 보이지 않는다.
[ ] hardware test 실행 중 손이 예상과 다르게 움직이거나 멈춰야 한다고 판단된다.
[ ] README 기준 실행 방법과 실제 환경이 맞지 않는다.
```

### 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C11_amazinghand_control_repo_test_<name>.md
```

### 보고서 템플릿

````markdown
# C11 AmazingHandControl Repo Test - <name>

## 실행 정보

- Date:
- Operator:
- Repo path:
- AmazingHandControl commit:
- Python version:
- Serial port:

## 실행한 명령

```bash
<실제로 실행한 명령>
```

## 결과

| 항목 | 결과 | 근거 |
|---|---|---|
| dependency install | pass/fail | output summary |
| non-hardware pytest | pass/fail | pytest summary |
| CLI list/run | pass/fail/blocked | command output |
| hardware pytest 또는 직접 CLI 실행 | pass/fail/blocked | command output / 관찰 기록 |

## 판단

```text
[ ] AmazingHandControl repo의 프로그램을 이 컴퓨터에서 사용할 수 있다.
[ ] 아직 사용할 수 없다. 이유:
[ ] C12에서 hand 쪽 parity를 포함해도 된다.
[ ] C12에서는 hand를 제외하고 arm motor parity만 본다. 이유:
```
````

### 승인 기준

```text
[ ] upstream repo commit과 실행 환경이 기록되어 있다.
[ ] hardware 없는 test 결과가 있다.
[ ] 실제 hand 연결 테스트를 했다면 실행 명령과 관찰 결과가 있다.
[ ] 실패했다면 dependency/port/hardware/프로그램 중 어느 쪽 문제인지 후보가 기록되어 있다.
```
