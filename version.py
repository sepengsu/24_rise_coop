import os
from git import Repo

repos = {
    "image_common": "https://github.com/ros-perception/image_common.git",
    "vision_opencv": "https://github.com/ros-perception/vision_opencv.git",
    "geometry2": "https://github.com/ros/geometry2.git",
    "common_msgs": "https://github.com/ros/common_msgs.git",
    "perception_pcl": "https://github.com/ros-perception/perception_pcl.git",
}

branches = {
    "image_common": "kinetic",
    "vision_opencv": "kinetic",
    "geometry2": "kinetic-devel",
    "common_msgs": "kinetic-devel",
    "perception_pcl": "kinetic",
}

commit_hashes = {}

for repo_name, repo_url in repos.items():
    repo_dir = f"/tmp/{repo_name}"
    if not os.path.exists(repo_dir):
        repo = Repo.clone_from(repo_url, repo_dir)
    else:
        repo = Repo(repo_dir)
    repo.git.checkout(branches[repo_name])
    commit_hash = repo.head.commit.hexsha
    commit_hashes[repo_name] = commit_hash

for repo_name, commit_hash in commit_hashes.items():
    print(f"{repo_name}: {commit_hash}")

