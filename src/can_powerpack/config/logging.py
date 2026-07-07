import os
import csv
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

def msg_to_dict(msg):
    """
    ROS 2 메시지 객체를 재귀적으로 탐색하여 딕셔너리 형태로 변환합니다.
    """
    if not hasattr(msg, '__slots__'):
        return msg
    
    result = {}
    for slot in msg.__slots__:
        value = getattr(msg, slot)
        # 중첩된 메시지(예: Header 내부의 stamp) 처리
        if hasattr(value, '__slots__'):
            result[slot] = msg_to_dict(value)
        # 리스트나 numpy 배열(예: 센서 데이터 배열)은 문자열로 변환
        elif isinstance(value, (list, tuple)) or type(value).__name__ == 'ndarray':
            result[slot] = str(value)
        else:
            result[slot] = value
    return result

def flatten_dict(d, parent_key='', sep='_'):
    """
    중첩된 딕셔너리를 평탄화(Flatten)하여 단일 깊이의 딕셔너리로 만듭니다.
    예: {'header': {'frame_id': 'base_link'}} -> {'header_frame_id': 'base_link'}
    """
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)

def extract_bag_to_csv(bag_dir, output_dir):
    """
    ROS 2 Bag 파일을 읽어 토픽별로 CSV 파일을 생성합니다.
    """
    # 출력 디렉토리 생성
    os.makedirs(output_dir, exist_ok=True)
    
    # 토픽별로 저장할 CSV 파일 객체와 Writer를 관리할 딕셔너리
    csv_files = {}
    csv_writers = {}
    
    print(f"[{bag_dir}] 데이터를 읽는 중입니다...")

    try:
        with Reader(bag_dir) as reader:
            # 타임스탬프 순서대로 메시지 순회
            for connection, timestamp, rawdata in reader.messages():
                topic_name = connection.topic
                msg_type = connection.msgtype
                
                # 메시지 역직렬화 (Deserialize)
                msg = deserialize_cdr(rawdata, msg_type)
                
                # 메시지를 평탄화된 딕셔너리로 변환 (시간 정보 추가)
                msg_dict = flatten_dict(msg_to_dict(msg))
                msg_dict['timestamp'] = timestamp # 나노초(ns) 단위 타임스탬프
                
                # 해당 토픽의 CSV 파일이 아직 열리지 않았다면 생성
                if topic_name not in csv_writers:
                    # 토픽 이름의 슬래시(/)를 밑줄(_)로 변경하여 파일명 생성
                    safe_topic_name = topic_name.strip('/').replace('/', '_')
                    csv_path = os.path.join(output_dir, f"{safe_topic_name}.csv")
                    
                    # 파일 열기 및 헤더 작성
                    f = open(csv_path, mode='w', newline='', encoding='utf-8')
                    csv_files[topic_name] = f
                    
                    # 타임스탬프를 첫 번째 열로 강제 지정
                    headers = ['timestamp'] + [k for k in msg_dict.keys() if k != 'timestamp']
                    writer = csv.DictWriter(f, fieldnames=headers)
                    writer.writeheader()
                    csv_writers[topic_name] = writer
                
                # 데이터 행 작성
                csv_writers[topic_name].writerow(msg_dict)
                
    finally:
        # 열려있는 모든 CSV 파일 안전하게 닫기
        for f in csv_files.values():
            f.close()
            
    print(f"변환 완료! 결과물은 [{output_dir}] 폴더에 저장되었습니다.")

if __name__ == "__main__":
    # TODO: 변환할 Bag 파일이 있는 폴더 경로와 결과를 저장할 폴더 경로를 지정하세요.
    BAG_DIRECTORY = './my_rosbag_data'  # db3 파일이 들어있는 디렉토리 경로
    OUTPUT_DIRECTORY = './csv_output'   # CSV가 저장될 경로
    
    extract_bag_to_csv(BAG_DIRECTORY, OUTPUT_DIRECTORY)