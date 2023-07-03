pipeline {
    agent any

    options {
        checkoutToSubdirectory('qs_drone_management')
    }

    stages {
        stage('Clean'){
            steps {
               dir("qs_drone_management") {
                    sh 'rm -rf .git env .gitignore Jenkinsfile Dockerfile docker-compose.yml README.md'
               }
            }
        }

        stage('Build / Test') {
            steps {
                echo 'Testing..'
            }
        }
        
        stage('create venv') {
            steps {
                sh 'python -m venv qs_drone_management_env'
            }
        }

        stage('Deploy') {

            environment {
                CREDENTIALS_ID = "${env.BRANCH_NAME == 'master' ? "credentiale_jenkins" : "testing_env"}"
            }

            steps {

                withPythonEnv('python') {
                    echo 'Installing pip'
                    sh 'curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python get-pip.py && rm get-pip.py'

                    echo 'Upgrading pip...'
                    sh 'pip install --upgrade pip'

                    echo 'Install ansible'
                    sh 'pip install wheel ansible docker docker-compose'

                    echo 'Install Docker Module and Community module for Ansible'
                    sh 'ansible-galaxy collection install community.docker && ansible-galaxy collection install community.general'
                    echo "Cloning Ansible script"

                    dir("qs_drone_management") {
                        dir('automations'){
                            checkout([$class: 'GitSCM',
                                    branches: [[name: "master"]],
                                    extensions: [],
                                    userRemoteConfigs: [[credentialsId: 'ID_AICI', // Deploy User ID
                                    url: 'https://github.com/AndreiMoiceanu29/DSD-Drone_Management.git']]]
                            )
                            echo "Move files to automations folder"
                            sh 'mkdir roles/repository/files/service'
                            sh 'mv ../drone_management/* roles/repository/files/service/'
                            withCredentials(
                                [
                                    usernamePassword(credentialsId: "credentiale-server", passwordVariable: 'pass', usernameVariable: 'usr')
                                ]
                            ) {
                                withCredentials([sshUserPrivateKey(credentialsId: "credentiale-ssh", keyFileVariable: 'prod_server')]) {
                                    sh 'ansible-playbook playbook.yml -i hosts --private-key ${prod_server} -u jenkins --ssh-common-args="-o StrictHostKeyChecking=no" -e "build_version=${BUILD_DISPLAY_NAME} branch_name=${BRANCH_NAME} ansible_become_password=${pass}"'
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    post {
        always {
            echo "Cleaning working directory"
            sh "rm -rf *"
        }
    }
}