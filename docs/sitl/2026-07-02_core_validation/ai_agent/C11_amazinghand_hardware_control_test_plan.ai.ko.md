# AI 보조 C11 - AmazingHandControl upstream 테스트 실행

## 역할

사람이 AmazingHandControl repo를 직접 설치하고 실행할 때, AI는 README/테스트 구조 정리, 명령 출력 해석, 산출물 작성 보조만 한다.

이 C11에서는 이 repo 안에 새 adapter나 control wrapper를 만들지 않는다.

## 확인할 자료

```text
docs/sitl/2026-07-02_core_validation/C11_amazinghand_hardware_control_test_plan.ko.md
/tmp/AmazingHandControl/README.md
/tmp/AmazingHandControl/pyproject.toml
/tmp/AmazingHandControl/requirements.txt
/tmp/AmazingHandControl/requirements-dev.txt
/tmp/AmazingHandControl/tests/conftest.py
/tmp/AmazingHandControl/tests/test_system_hardware.py
/tmp/AmazingHandControl/tests/test_cmd_hardware.py
```

## 도울 수 있는 일

```text
[ ] README에서 설치 명령과 실행 명령을 요약한다.
[ ] hardware 없이 실행되는 test와 hardware test를 구분해준다.
[ ] `pytest` 실패 로그를 읽고 dependency/port/permission/test failure 중 어디에 가까운지 정리한다.
[ ] 사람이 실행한 명령과 결과를 C11 산출물 양식으로 정리한다.
```

## 직접 하지 말 것

```text
[ ] 새 AmazingHand adapter를 이 repo에 구현하지 않는다.
[ ] upstream repo의 테스트 의도를 바꿔 해석하지 않는다.
[ ] 사람이 실행하지 않은 hardware motion을 성공으로 기록하지 않는다.
[ ] upstream repo 실행 결과와 별개의 health checklist를 새로 만들지 않는다.
```

## 참고 명령

```bash
cd /tmp/AmazingHandControl
git rev-parse HEAD
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-dev.txt
pytest -q
python amazing_hand_cmd.py --list
pytest tests/test_system_hardware.py tests/test_cmd_hardware.py --hardware --port /dev/ttyACM0
```

## 산출물에 정리할 것

```text
[ ] AmazingHandControl commit.
[ ] Python/dependency 설치 결과.
[ ] non-hardware test 결과.
[ ] CLI 또는 hardware test 실행 결과.
[ ] 실패했다면 다음에 사람이 확인할 항목.
```
