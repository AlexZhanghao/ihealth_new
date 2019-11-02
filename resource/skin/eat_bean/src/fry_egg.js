// 找到在html中定义的div
var startdiv=document.getElementById('startdiv')
var maindiv=document.getElementById('maindiv')
var score=document.getElementById('score')
var totalscore=document.getElementById('totalscore')
var enddiv=document.getElementById('enddiv')

var log = console.log


// 加入一个盘子
window.m_plate = null

// 加入一个平底锅
window.m_pan = null

// 加入一个鸡蛋篮子
window.m_egg_basket = null

// 加入一个只手
window.m_hand = null

// 做好了的鸡蛋
window.m_cooked_egg = null

// 得分
var scores = 0

var tid

// 定义游戏开始的函数，这时候应该显示maindiv，隐藏startdiv
function begin(){
	startdiv.style.display='none'
	maindiv.style.display='block'
	
	// // 添加盘子，平底锅和鸡蛋篮子等不动的物件
	// m_pan = new Pan(10, 80, "image/pan.png")
	
	// 添加一个能动的手
	m_hand = new Hand(700, 581, "image/hand.png")
	
	tid = setInterval(fry_egg, 200)
}


// 定义擦窗户成功的判定函数，现在以抹布从窗户外进入窗户内，然后又出窗户定义为一次擦窗户
var beans=[];
var create_bean=1;
var fry_egg = function() {
	var hand = window.m_hand
	//var pan = window.m_pan
	
	if(create_bean==1){
		beans.push(new build_bean(10, 80, "image/pan.png"))
		create_bean--;
		
	}
	
	// 手碰到平底锅后平底锅消失
	var beanlen = beans.length;	
	if (A_in_B(hand, beans[beanlen-1])) {
	scores+=100
	score.innerHTML = scores
  	for (var i = 0; i < beanlen; i++) {       		
		    maindiv.removeChild(beans[0].node);//删除节点
            beans.splice(i,1);	
            beanlen--;			          
	    }
		create_bean++;
	}
	
}

//随机生成豆子
function build_bean(x, y, imagesrc){
	// var creat=random1(0,0)+random2(0,0)+random3(0,0);
	// if(creat==0){
	    // Pan.call(this,100,0,imagesrc);
	// }
	// else if(creat==1) {
		// Pan.call(this,10,80,imagesrc);
	// }
    // else if(creat==2){
		// Pan.call(this,500,0,imagesrc);
	// }
	// else if(creat==3){
		// Pan.call(this,700,0,imagesrc);
	// }
	Pan.call(this,Math.random()*600,Math.random()*500,imagesrc);
}
// function random1(x,y){
	// return Math.round(Math.random());
// }
// function random2(x,y){
	// return Math.round(Math.random());
// }
// function random3(x,y){
	// return Math.round(Math.random());
// }
function random(x,y){
	return Math.floor(Math.random()*(y-x)+x);
}

// A和B相离函数
var A_disjoint_with_B = function(A, B) {
	var rag_w = A.node.clientWidth // 抹布的宽度
	var rag_h = A.node.clientHeight // 抹布的高度
	var rag_l = A.x // 抹布的最左边
	var rag_t = A.y // 抹布的最上边
	
	var win_w = B.node.clientWidth // 窗户的宽度
	var win_h = B.node.clientHeight // 窗户的高度
	var win_l = B.x // 窗户的最左边
	var win_t = B.y // 窗户的最右边
	
	if (rag_l > win_l + win_w || rag_l + rag_w < win_l || 
		rag_t + rag_h < win_t || rag_t > win_t + win_h) {
			return true
	}
	
	return false
}

// A在B内部
var A_in_B = function(A, B) {
	var rag_w = A.node.clientWidth // 抹布的宽度
	var rag_h = A.node.clientHeight // 抹布的高度
	var rag_l = A.x // 抹布的最左边
	var rag_t = A.y // 抹布的最上边
	
	var win_w = B.node.clientWidth // 窗户的宽度
	var win_h = B.node.clientHeight // 窗户的高度
	var win_l = B.x // 窗户的最左边
	var win_t = B.y // 窗户的最右边

	if (rag_l >= win_l && rag_l + rag_w <= win_l + win_w &&
		rag_t >= win_t && rag_t + rag_h <= win_t + win_h) {
		return true
	}
	
	return false
}

// 和C++端对应的接口，用来区分不同的游戏
function getGameType() {
	return '煎鸡蛋'	
}

function getWidth() {
	var x = document.documentElement.clientWidth;//selfplane.x*2;
	return x
};

function getHeight() {
	return document.documentElement.clientHeight;// selfplane.y
};

function getScore() {
	return scores	
}

// 控制手运动到x, y
var selfmove = function(x, y) {
	hand = window.m_hand
	hand.move_to(x, y)
}



// // 设置对键盘的监听事件
// document.onkeydown = function(event) {
	// var e = event
	// var k = e.keyCode
	// m_hand= window.m_hand
	// m_egg_basket = window.m_egg_basket
	
	// var speed = 20
	// if (k == 38) {
		// // up
		// m_hand.move_to(m_hand.x, m_hand.y-speed)
	// } else if (k == 40) {
		// // down		
		// m_hand.move_to(m_hand.x, m_hand.y + speed)
	// } else if (k == 37) {
		// // left	
		// m_hand.move_to(m_hand.x-speed, m_hand.y)
	// } else if (k == 39) {
		// // right	
		// m_hand.move_to(m_hand.x + speed, m_hand.y)
	// }
// }




