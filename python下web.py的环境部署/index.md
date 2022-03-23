# Python下web.py的环境部署


Tags: #python #webpy #环境部署  
Links: [[Python下web.py使用ssl加密]], [[Python实现开发模式下公众号自动回复]]

---

# Python下web.py的环境部署

## prerequisites:
当[[web.py]]被用于对接微信公众号时，必要的前置为：
- python2.7版本以上 `yum install python3`
- [[pip]] `pip3 install --upgrade pip`
- web.py `python3 -m pip install web.py`
- libxml2
- libxslt
- lxml `python3 -m pip install lxml`

## 部署：
**[[Supervisor]]**:
Supervisord是一个守护进程的工具，当进程意外终止或服务器掉电重启后，进程可以继续自动运行。同时supervisord自带监控界面，可以通过浏览器灵活地查看、操作。

1. 安装：
`ubantu: apt-get install supervisor`
`centos: yum install supervisor`
2. 配置supervisord.conf文件：
centos系统下该文件在/etc/目录下，ubantu系统下该文件在/etc/supervisor/目录下：
`ubantu: sudo vim /etc/supervisor/supervisord.conf`
`centos: sudo vim /etc/supervisord.conf`
3. 配置：
在文件末尾添加如下几行：
 ``` conf
 [program:myProgramName]
 command=python3 /home/myname/test.py 80
 autostart=true
 autorestart=true
 stdout_logfile=/home/myname/test.log
 ```
 最后一行表示程序打印出的信息都记录在该test.log文件内。
 4. 运行：
 启动supervisor和自己的程序：
 `supervisord -c /etc/supervisord.conf`
 意为按照配置文件执行supervisor
 5. 常用命令：
 `service supervisor stop #停止supervisor服务`  
 `service supervisor start #启动supervisor服务  `  
 `supervisorctl shutdown #关闭所有任务`  
 `supervisorctl stop|start program_name #启动或停止某项服务     `  
 `supervisorctl status #查看所有任务状态`
