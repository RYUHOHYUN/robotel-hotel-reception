# GitHub Upload Checklist

## 반드시 제외

- [ ] `.env`
- [ ] OpenAI API key와 기타 token
- [ ] `*.db`, 실제 예약·고객 데이터
- [ ] 호텔 DB Excel 파일
- [ ] 여권 원본 사진과 crop 이미지
- [ ] 실시간 얼굴 촬영 이미지
- [ ] `build/`, `install/`, `log/`
- [ ] `__pycache__/`, `*.pyc`
- [ ] 장비별 임시 output

## 코드 확인

- [ ] README의 역할 범위가 실제 기여와 일치하는가
- [ ] 팀 전체 기능을 개인 단독 구현으로 표현하지 않았는가
- [ ] 하드코딩된 IP와 좌표가 공개 가능한가
- [ ] 실제 로봇 좌표가 다른 환경에서 위험하다는 경고가 있는가
- [ ] 외부 모델 파일의 재배포 라이선스를 확인했는가
- [ ] Doosan, OnRobot 관련 SDK·wrapper 재배포 조건을 확인했는가

## 미디어 추가 전

- [ ] 화면에 여권번호, 이름, 생년월일이 보이지 않는가
- [ ] 얼굴이 포함된 경우 당사자의 공개 동의를 받았는가
- [ ] 키오스크 화면에 예약번호나 DB 정보가 보이지 않는가
- [ ] 영상의 음성에 개인정보가 포함되지 않는가
- [ ] 파일 크기가 GitHub 제한에 적합한가

## 추천 업로드 순서

```bash
git init
git add README.md docs config interfaces src media .gitignore .env.example requirements.txt
git status
git diff --cached
git commit -m "Document ROBOTEL vision and robot control contribution"
git branch -M main
git remote add origin <YOUR_REPOSITORY_URL>
git push -u origin main
```

`git status`와 `git diff --cached`에서 DB, `.env`, 여권·얼굴 이미지가 포함되지 않았는지 마지막으로 확인하세요.
