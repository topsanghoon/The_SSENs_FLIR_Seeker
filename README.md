# 🧩 PetaLinux Build & Device Tree 재생성 가이드

이 문서는 **Vivado 하드웨어(XSA)** 변경 후 PetaLinux에서  
**디바이스 트리(Device Tree)** 및 **이미지 재빌드**를 수행하는 절차를 정리한 것입니다.

---

## 🚀 기본 빌드 절차

```bash
# 1. Vivado에서 export한 하드웨어(XSA) 반영
petalinux-config --get-hw-description project-spec/hw-description --silentconfig

# 2. 전체 빌드 수행
petalinux-build
빌드 산출물은 아래 경로에서 확인할 수 있습니다.

bash
코드 복사
{workspace}/images/linux/
⚠️ 부팅 불가 시 조치 절차
하드웨어(XSA) 변경 후 부팅이 실패하거나 Device Tree 충돌이 발생한 경우,
다음 순서대로 클린 빌드를 수행합니다.

1️⃣ 기존 Device Tree 정리
bash
코드 복사
petalinux-build -c device-tree -x cleanall
rm -rf components/plnx_workspace/device-tree
2️⃣ 새 XSA 반영
Vivado에서 생성한 최신 .xsa 파일이 있는 폴더를 지정합니다.

bash
코드 복사
petalinux-config --get-hw-description <XSA_폴더> --silentconfig
메뉴가 열리면 Save → Exit 으로 종료합니다.
(이 과정에서 새 하드웨어 설정이 반영됩니다.)

3️⃣ Device Tree만 우선 재생성
bash
코드 복사
petalinux-build -c device-tree
4️⃣ 전체 이미지 재빌드
bash
코드 복사
petalinux-build
📦 최종 산출물 위치
파일	설명	경로
image.ub	통합 커널 + 루트FS 이미지	images/linux/
BOOT.BIN	FSBL + 비트스트림 + U-Boot	images/linux/
system.dtb	Device Tree Blob	images/linux/

🧠 참고
--silentconfig 옵션은 GUI 메뉴 없이 기존 설정을 그대로 적용합니다.

XSA 파일이 변경되면 항상 petalinux-config --get-hw-description 부터 다시 실행하세요.

Device Tree 수정 시에는 -c device-tree 타겟만 별도로 빌드해도 충분합니다.

