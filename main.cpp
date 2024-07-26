#include <Novice.h>
#include <Vector3.h>
#include <Matrix4x4.h>
#include <cstdint>
#include <assert.h>
#include <cmath>
#include <math.h>
#include <imgui.h>

const char kWindowTitle[] = "LC1B_06_オノ_ケンジピエール_タイトル";

static const int kWindowWidth = 1280;
static const int kWindowHeight = 720;

struct AABB {
	Vector3 min;
	Vector3 max;
};

struct Segment {
	Vector3 origin;
	Vector3 diff;
};

float cot(float x) {
	float cot;
	cot = 1 / tanf(x);
	return cot;
}

Vector3 Add(const Vector3& v1, const Vector3& v2) {
	Vector3 result;
	result.x = v1.x + v2.x;
	result.y = v1.y + v2.y;
	result.z = v1.z + v2.z;
	return result;
}

Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 result;
	result = {
		(1 / aspectRatio) * cot(fovY / 2),0,0,0,
		0,cot(fovY / 2),0,0,
		0,0,farClip / (farClip - nearClip),1,
		0,0,(-nearClip * farClip) / (farClip - nearClip),0
	};
	return result;
}

Matrix4x4 MakeVieportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result;
	result = {
		width / 2,0,0,0,
		0,-height / 2,0,0,
		0,0,maxDepth - minDepth,0,
		left + (width / 2),top + (height / 2),minDepth,1
	};
	return result;
}

Matrix4x4 MultiplyMat4(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result = {};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				result.m[i][j] += m1.m[i][k] * m2.m[k][j];
			}
		}
	}
	return result;
}

Matrix4x4 MakeRotateMatrix(const Vector3& rotate) {
	Matrix4x4 rotateX;
	rotateX = {
	1,0,0,0,
		0,std::cos(rotate.x),std::sin(rotate.x),0,
		0,-std::sin(rotate.x),std::cos(rotate.x),0,
		0,0,0,1
	};
	Matrix4x4 rotateY;
	rotateY = {
	std::cos(rotate.y),0,-std::sin(rotate.y),0,
		0,1,0,0,
		std::sin(rotate.y),0,std::cos(rotate.y),
		0,0,0,0,1
	};
	Matrix4x4 rotateZ;
	rotateZ = {
	std::cos(rotate.z),std::sin(rotate.z),0,0,
		-std::sin(rotate.z),std::cos(rotate.z),0,0,
		0,0,1,0,
		0,0,0,1
	};
	Matrix4x4 result = MultiplyMat4(rotateX, MultiplyMat4(rotateY, rotateZ));
	return result;
}

Matrix4x4 Inverse(const Matrix4x4& m) {
	float determinant =
		+m.m[0][0] * m.m[1][1] * m.m[2][2] * m.m[3][3]
		+ m.m[0][0] * m.m[1][2] * m.m[2][3] * m.m[3][1]
		+ m.m[0][0] * m.m[1][3] * m.m[2][1] * m.m[3][2]

		- m.m[0][0] * m.m[1][3] * m.m[2][2] * m.m[3][1]
		- m.m[0][0] * m.m[1][2] * m.m[2][1] * m.m[3][3]
		- m.m[0][0] * m.m[1][1] * m.m[2][3] * m.m[3][2]

		- m.m[0][1] * m.m[1][0] * m.m[2][2] * m.m[3][3]
		- m.m[0][2] * m.m[1][0] * m.m[2][3] * m.m[3][1]
		- m.m[0][3] * m.m[1][0] * m.m[2][1] * m.m[3][2]

		+ m.m[0][3] * m.m[1][0] * m.m[2][2] * m.m[3][1]
		+ m.m[0][2] * m.m[1][0] * m.m[2][1] * m.m[3][3]
		+ m.m[0][1] * m.m[1][0] * m.m[2][3] * m.m[3][2]

		+ m.m[0][1] * m.m[1][2] * m.m[2][0] * m.m[3][3]
		+ m.m[0][2] * m.m[1][3] * m.m[2][0] * m.m[3][1]
		+ m.m[0][3] * m.m[1][1] * m.m[2][0] * m.m[3][2]

		- m.m[0][3] * m.m[1][2] * m.m[2][0] * m.m[3][1]
		- m.m[0][2] * m.m[1][1] * m.m[2][0] * m.m[3][3]
		- m.m[0][1] * m.m[1][3] * m.m[2][0] * m.m[3][2]

		- m.m[0][1] * m.m[1][2] * m.m[2][3] * m.m[3][0]
		- m.m[0][2] * m.m[1][3] * m.m[2][1] * m.m[3][0]
		- m.m[0][3] * m.m[1][1] * m.m[2][2] * m.m[3][0]

		+ m.m[0][3] * m.m[1][2] * m.m[2][1] * m.m[3][0]
		+ m.m[0][2] * m.m[1][1] * m.m[2][3] * m.m[3][0]
		+ m.m[0][1] * m.m[1][3] * m.m[2][2] * m.m[3][0];

	Matrix4x4 result = {};
	float recpDeterminant = 1.0f / determinant;
	result.m[0][0] = (m.m[1][1] * m.m[2][2] * m.m[3][3] + m.m[1][2] * m.m[2][3] * m.m[3][1] +
		m.m[1][3] * m.m[2][1] * m.m[3][2] - m.m[1][3] * m.m[2][2] * m.m[3][1] -
		m.m[1][2] * m.m[2][1] * m.m[3][3] - m.m[1][1] * m.m[2][3] * m.m[3][2]) * recpDeterminant;
	result.m[0][1] = (-m.m[0][1] * m.m[2][2] * m.m[3][3] - m.m[0][2] * m.m[2][3] * m.m[3][1] -
		m.m[0][3] * m.m[2][1] * m.m[3][2] + m.m[0][3] * m.m[2][2] * m.m[3][1] +
		m.m[0][2] * m.m[2][1] * m.m[3][3] + m.m[0][1] * m.m[2][3] * m.m[3][2]) * recpDeterminant;
	result.m[0][2] = (m.m[0][1] * m.m[1][2] * m.m[3][3] + m.m[0][2] * m.m[1][3] * m.m[3][1] +
		m.m[0][3] * m.m[1][1] * m.m[3][2] - m.m[0][3] * m.m[1][2] * m.m[3][1] -
		m.m[0][2] * m.m[1][1] * m.m[3][3] - m.m[0][1] * m.m[1][3] * m.m[3][2]) * recpDeterminant;
	result.m[0][3] = (-m.m[0][1] * m.m[1][2] * m.m[2][3] - m.m[0][2] * m.m[1][3] * m.m[2][1] -
		m.m[0][3] * m.m[1][1] * m.m[2][2] + m.m[0][3] * m.m[1][2] * m.m[2][1] +
		m.m[0][2] * m.m[1][1] * m.m[2][3] + m.m[0][1] * m.m[1][3] * m.m[2][2]) * recpDeterminant;

	result.m[1][0] = (-m.m[1][0] * m.m[2][2] * m.m[3][3] - m.m[1][2] * m.m[2][3] * m.m[3][0] -
		m.m[1][3] * m.m[2][0] * m.m[3][2] + m.m[1][3] * m.m[2][2] * m.m[3][0] +
		m.m[1][2] * m.m[2][0] * m.m[3][3] + m.m[1][0] * m.m[2][3] * m.m[3][2]) * recpDeterminant;
	result.m[1][1] = (m.m[0][0] * m.m[2][2] * m.m[3][3] + m.m[0][2] * m.m[2][3] * m.m[3][0] +
		m.m[0][3] * m.m[2][0] * m.m[3][2] - m.m[0][3] * m.m[2][2] * m.m[3][0] -
		m.m[0][2] * m.m[2][0] * m.m[3][3] - m.m[0][0] * m.m[2][3] * m.m[3][2]) * recpDeterminant;
	result.m[1][2] = (-m.m[0][0] * m.m[1][2] * m.m[3][3] - m.m[0][2] * m.m[1][3] * m.m[3][0] -
		m.m[0][3] * m.m[1][0] * m.m[3][2] + m.m[0][3] * m.m[1][2] * m.m[3][0] +
		m.m[0][2] * m.m[1][0] * m.m[3][3] + m.m[0][0] * m.m[1][3] * m.m[3][2]) * recpDeterminant;
	result.m[1][3] = (m.m[0][0] * m.m[1][2] * m.m[2][3] + m.m[0][2] * m.m[1][3] * m.m[2][0] +
		m.m[0][3] * m.m[1][0] * m.m[2][2] - m.m[0][3] * m.m[1][2] * m.m[2][0] -
		m.m[0][2] * m.m[1][0] * m.m[2][3] - m.m[0][0] * m.m[1][3] * m.m[2][2]) * recpDeterminant;

	result.m[2][0] = (m.m[1][0] * m.m[2][1] * m.m[3][3] + m.m[1][1] * m.m[2][3] * m.m[3][0] +
		m.m[1][3] * m.m[2][0] * m.m[3][1] - m.m[1][3] * m.m[2][1] * m.m[3][0] -
		m.m[1][1] * m.m[2][0] * m.m[3][3] - m.m[1][0] * m.m[2][3] * m.m[3][1]) * recpDeterminant;
	result.m[2][1] = (-m.m[0][0] * m.m[2][1] * m.m[3][3] - m.m[0][1] * m.m[2][3] * m.m[3][0] -
		m.m[0][3] * m.m[2][0] * m.m[3][1] + m.m[0][3] * m.m[2][1] * m.m[3][0] +
		m.m[0][1] * m.m[2][0] * m.m[3][3] + m.m[0][0] * m.m[2][3] * m.m[3][1]) * recpDeterminant;
	result.m[2][2] = (m.m[0][0] * m.m[1][1] * m.m[3][3] + m.m[0][1] * m.m[1][3] * m.m[3][0] +
		m.m[0][3] * m.m[1][0] * m.m[3][1] - m.m[0][3] * m.m[1][1] * m.m[3][0] -
		m.m[0][1] * m.m[1][0] * m.m[3][3] - m.m[0][0] * m.m[1][3] * m.m[3][1]) * recpDeterminant;
	result.m[2][3] = (-m.m[0][0] * m.m[1][1] * m.m[2][3] - m.m[0][1] * m.m[1][3] * m.m[2][0] -
		m.m[0][3] * m.m[1][0] * m.m[2][1] + m.m[0][3] * m.m[1][1] * m.m[2][0] +
		m.m[0][1] * m.m[1][0] * m.m[2][3] + m.m[0][0] * m.m[1][3] * m.m[2][1]) * recpDeterminant;

	result.m[3][0] = (-m.m[1][0] * m.m[2][1] * m.m[3][2] - m.m[1][1] * m.m[2][2] * m.m[3][0] -
		m.m[1][2] * m.m[2][0] * m.m[3][1] + m.m[1][2] * m.m[2][1] * m.m[3][0] +
		m.m[1][1] * m.m[2][0] * m.m[3][2] + m.m[1][0] * m.m[2][2] * m.m[3][1]) * recpDeterminant;
	result.m[3][1] = (m.m[0][0] * m.m[2][1] * m.m[3][2] + m.m[0][1] * m.m[2][2] * m.m[3][0] +
		m.m[0][2] * m.m[2][0] * m.m[3][1] - m.m[0][2] * m.m[2][1] * m.m[3][0] -
		m.m[0][1] * m.m[2][0] * m.m[3][2] - m.m[0][0] * m.m[2][2] * m.m[3][1]) * recpDeterminant;
	result.m[3][2] = (-m.m[0][0] * m.m[1][1] * m.m[3][2] - m.m[0][1] * m.m[1][2] * m.m[3][0] -
		m.m[0][2] * m.m[1][0] * m.m[3][1] + m.m[0][2] * m.m[1][1] * m.m[3][0] +
		m.m[0][1] * m.m[1][0] * m.m[3][2] + m.m[0][0] * m.m[1][2] * m.m[3][1]) * recpDeterminant;
	result.m[3][3] = (m.m[0][0] * m.m[1][1] * m.m[2][2] + m.m[0][1] * m.m[1][2] * m.m[2][0] +
		m.m[0][2] * m.m[1][0] * m.m[2][1] - m.m[0][2] * m.m[1][1] * m.m[2][0] -
		m.m[0][1] * m.m[1][0] * m.m[2][2] - m.m[0][0] * m.m[1][2] * m.m[2][1]) * recpDeterminant;

	return result;
}

Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 result;
	Matrix4x4 rotateM = MakeRotateMatrix(rotate);
	result = {
		scale.x * rotateM.m[0][0],scale.x * rotateM.m[0][1],scale.x * rotateM.m[0][2],0,
		scale.y * rotateM.m[1][0],scale.y * rotateM.m[1][1],scale.y * rotateM.m[1][2],0,
		scale.z * rotateM.m[2][0],scale.z * rotateM.m[2][1],scale.z * rotateM.m[2][2],0,
		translate.x,translate.y,translate.z,1
	};
	return result;
}

Vector3 Transform(Vector3 vector, Matrix4x4 matrix) {
	Vector3 result;
	result = {
		vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0],
		vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1],
		vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2]
	};
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + matrix.m[3][3];

	assert(w != 0.0f);
	result.x /= w;
	result.y /= w;
	result.z /= w;

	return result;
}

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {

	const float kGridHalfWidth = 2.0f;
	const uint32_t kSubdivision = 10;
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision);

	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		float x = -kGridHalfWidth + (xIndex * kGridEvery);
		unsigned int color = 0xAAAAAAFF;

		Vector3 start{ x,0.0f,-kGridHalfWidth };
		Vector3 end{ x,0.0f,kGridHalfWidth };

		Vector3 startScreen = Transform(Transform(start, viewProjectionMatrix), viewportMatrix);
		Vector3 endScreen = Transform(Transform(end, viewProjectionMatrix), viewportMatrix);

		if (x == 0.0f) {
			color = BLACK;
		}
		Novice::DrawLine(int(startScreen.x), int(startScreen.y), int(endScreen.x), int(endScreen.y), color);
	}
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		float z = -kGridHalfWidth + (zIndex * kGridEvery);
		unsigned int color = 0xAAAAAAFF;

		Vector3 start{ -kGridHalfWidth,0.0f,z };
		Vector3 end{ kGridHalfWidth,0.0f,z };

		Vector3 startScreen = Transform(Transform(start, viewProjectionMatrix), viewportMatrix);
		Vector3 endScreen = Transform(Transform(end, viewProjectionMatrix), viewportMatrix);

		if (z == 0.0f) {
			color = BLACK;
		}
		Novice::DrawLine(int(startScreen.x), int(startScreen.y), int(endScreen.x), int(endScreen.y), color);
	}
}

void DrawAABB(const AABB& aabb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	const int pointNum = 8;
	Vector3 point[pointNum]{
		{aabb.min.x,aabb.min.y,aabb.min.z},//---0
		{aabb.min.x,aabb.max.y,aabb.min.z},//-+-1
		{aabb.min.x,aabb.min.y,aabb.max.z},//--+2
		{aabb.min.x,aabb.max.y,aabb.max.z},//-++3
		{aabb.max.x,aabb.min.y,aabb.min.z},//+--4
		{aabb.max.x,aabb.max.y,aabb.min.z},//++-5
		{aabb.max.x,aabb.min.y,aabb.max.z},//+-+6
		{aabb.max.x,aabb.max.y,aabb.max.z}//+++7
	};
	Vector3 worldPoint[pointNum];
	for (int i = 0; i < pointNum; i++) {
		worldPoint[i] = Transform(Transform(point[i], viewProjectionMatrix), viewportMatrix);
	}

	Novice::DrawLine(int(worldPoint[0].x), int(worldPoint[0].y), int(worldPoint[1].x), int(worldPoint[1].y), color);//01
	Novice::DrawLine(int(worldPoint[0].x), int(worldPoint[0].y), int(worldPoint[2].x), int(worldPoint[2].y), color);//02
	Novice::DrawLine(int(worldPoint[0].x), int(worldPoint[0].y), int(worldPoint[4].x), int(worldPoint[4].y), color);//04
	Novice::DrawLine(int(worldPoint[1].x), int(worldPoint[1].y), int(worldPoint[3].x), int(worldPoint[3].y), color);//13
	Novice::DrawLine(int(worldPoint[1].x), int(worldPoint[1].y), int(worldPoint[5].x), int(worldPoint[5].y), color);//15
	Novice::DrawLine(int(worldPoint[2].x), int(worldPoint[2].y), int(worldPoint[3].x), int(worldPoint[3].y), color);//23
	Novice::DrawLine(int(worldPoint[2].x), int(worldPoint[2].y), int(worldPoint[6].x), int(worldPoint[6].y), color);//26
	Novice::DrawLine(int(worldPoint[3].x), int(worldPoint[3].y), int(worldPoint[7].x), int(worldPoint[7].y), color);//37
	Novice::DrawLine(int(worldPoint[4].x), int(worldPoint[4].y), int(worldPoint[5].x), int(worldPoint[5].y), color);//45
	Novice::DrawLine(int(worldPoint[4].x), int(worldPoint[4].y), int(worldPoint[6].x), int(worldPoint[6].y), color);//46
	Novice::DrawLine(int(worldPoint[5].x), int(worldPoint[5].y), int(worldPoint[7].x), int(worldPoint[7].y), color);//57
	Novice::DrawLine(int(worldPoint[6].x), int(worldPoint[6].y), int(worldPoint[7].x), int(worldPoint[7].y), color);//67

}

bool IsCollision(const AABB& aabb, const Segment& segment) {
	float txmin = (aabb.min.x - segment.origin.x) / segment.diff.x;
	float txmax = (aabb.max.x - segment.origin.x) / segment.diff.x;
	float tymin = (aabb.min.y - segment.origin.y) / segment.diff.y;
	float tymax = (aabb.max.y - segment.origin.y) / segment.diff.y;
	float tzmin = (aabb.min.z - segment.origin.z) / segment.diff.z;
	float tzmax = (aabb.max.z - segment.origin.z) / segment.diff.z;

	float tNearX = min(txmin, txmax);
	float tNearY = min(tymin, tymax);
	float tNearZ = min(tzmin, tzmax);
	float tFarX = max(txmin, txmax);
	float tFarY = max(tymin, tymax);
	float tFarZ = max(tzmin, tzmax);

	float tmin = max(max(tNearX, tNearY), tNearZ);

	float tmax = min(min(tFarX, tFarY), tFarZ);

	if (tmin <= tmax) {
		return true;
	}
	else {
		return false;
	}
}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	Segment segment{ {-0.7f,0.3f,0.0f},{2.0f,-0.5f,0.0f} };
	AABB aabb{ {-0.5f,-0.5f,-0.5f},{0.5f,0.5f,0.5f} };

	Vector3 rotate{ 0.0f,0.0f,0.0f };
	Vector3 translate{ 0.0f,0.0f,0.0f };

	Vector3 cameraTranslate{ 0.0f,1.9f,-6.49f };
	Vector3 cameraRotate{ 0.26f,0.0f,0.0f };

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		Matrix4x4 worldMatrix = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, rotate, translate);
		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 viewProjectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 worldViewProjectionMatrix = MultiplyMat4(worldMatrix, MultiplyMat4(viewMatrix, viewProjectionMatrix));
		Matrix4x4 viewportMatrix = MakeVieportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		ImGui::Begin("Window");
		ImGui::DragFloat3("aabb.min", &aabb.min.x, 0.01f);
		aabb.min.x = (std::min)(aabb.min.x, aabb.max.x);
		aabb.min.y = (std::min)(aabb.min.y, aabb.max.y);
		aabb.min.z = (std::min)(aabb.min.z, aabb.max.z);
		ImGui::DragFloat3("aabb.max", &aabb.max.x, 0.01f);
		aabb.max.x = (std::max)(aabb.min.x, aabb.max.x);
		aabb.max.y = (std::max)(aabb.min.y, aabb.max.y);
		aabb.max.z = (std::max)(aabb.min.z, aabb.max.z);
		ImGui::DragFloat3("segment.origin", &segment.origin.x, 0.01f);
		ImGui::DragFloat3("segment.diff", &segment.diff.x, 0.01f);
		ImGui::DragFloat3("c.t", &cameraTranslate.x, 0.01f);
		ImGui::DragFloat3("c.r", &cameraRotate.x, 0.01f);
		ImGui::End();

		Vector3 start = Transform(Transform(segment.origin, worldViewProjectionMatrix), viewportMatrix);
		Vector3 end = Transform(Transform(Add(segment.origin, segment.diff), worldViewProjectionMatrix), viewportMatrix);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		DrawGrid(worldViewProjectionMatrix, viewportMatrix);

		Novice::DrawLine(int(start.x), int(start.y), int(end.x), int(end.y), WHITE);

		if (IsCollision(aabb, segment)) {
			DrawAABB(aabb, worldViewProjectionMatrix, viewportMatrix, RED);
		}
		else {
			DrawAABB(aabb, worldViewProjectionMatrix, viewportMatrix, WHITE);
		}


		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
